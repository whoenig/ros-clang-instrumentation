#include "llvm/Pass.h"
#include "llvm/IR/Function.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/InstrTypes.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/IR/IRBuilder.h"
#include "llvm/Transforms/Utils/BasicBlockUtils.h"
#include "llvm/IR/Module.h"
#include "llvm/Transforms/Utils/Cloning.h"
#include "llvm/IR/DebugInfoMetadata.h"
#include "llvm/ADT/SCCIterator.h"
#include <cxxabi.h>
#include <fstream>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/kruskal_min_spanning_tree.hpp>
#include <boost/graph/graphviz.hpp>

using namespace llvm;
using namespace std;

using namespace boost;

// test with:
// clang -Xclang -load -Xclang ballLarus/libballLarusPass.so ../example.c

inline std::string demangle(const std::string& name)
{
  int status = -1;

  std::unique_ptr<char, void(*)(void*)> res { abi::__cxa_demangle(name.c_str(), NULL, NULL, &status), std::free };
  return (status == 0) ? res.get() : std::string(name);
}

void dumpToFile(Value* f, const std::string& fileName)
{
  std::error_code EC;
  raw_fd_ostream str(fileName, EC, (llvm::sys::fs::OpenFlags)4);//llvm::sys::fs::F_Text);
  str << *f;
}

//////////////////////
// Graph stuff

struct Vertex
{
  Vertex()
    : blocks()
    , numPaths(0)
  {
  }

  std::vector<BasicBlock*> blocks;
  uint32_t numPaths;
};

struct Edge
{
  Edge()
    : val(0)
    , chord(false)
    , increment(0)
    , terminatorInst(nullptr)
    , terminatorIdx(-1)
    , insertPt(nullptr)
  {
  }

  uint32_t val;
  bool chord;
  int32_t increment;
  TerminatorInst* terminatorInst;
  size_t terminatorIdx;
  Instruction* insertPt;
};

typedef boost::adjacency_list<
        boost::vecS, boost::vecS, boost::bidirectionalS,
        Vertex, Edge>
        graph_t;

typedef boost::graph_traits<graph_t>::vertex_descriptor vertex_t;
typedef boost::graph_traits<graph_t>::edge_descriptor edge_t;

unsigned line(vertex_t v, const graph_t& graph) {
  for (const auto& I : *graph[v].blocks.at(0)) {
    if (const DILocation *loc = I.getDebugLoc()) {
      unsigned line = loc->getLine();
      StringRef file = loc->getFilename();
      StringRef dir = loc->getDirectory();
      return line;
    }
  }
  return -1;
}

class myVertexWriter {
public:
  explicit myVertexWriter(
    const graph_t& graph)
    : m_graph(graph)
  {
  }

  void operator()(
    std::ostream &out,
    const vertex_t &v) const
  {
    // find first instruction in first block with line information
    for (auto& I : *m_graph[v].blocks.at(0)) {
      if (DILocation *loc = I.getDebugLoc()) {
        unsigned line = loc->getLine();
        StringRef file = loc->getFilename();
        StringRef dir = loc->getDirectory();
        out << "[label=\"" << line << ", numPaths:" << m_graph[v].numPaths << "\"]";
        return;
      }
    }
  }
private:
  const graph_t& m_graph;
};

class myEdgeWriter {
public:
  explicit myEdgeWriter(
      const graph_t& graph)
      : m_graph(graph)
  {
  }

    void operator()(
      std::ostream &out,
      const edge_t &e) const
    {
       out << "[label=\"" << m_graph[e].val << "," << m_graph[e].chord << "," << m_graph[e].increment << "\"]";
    }
private:
    const graph_t& m_graph;
};

void saveDotFile(
  const graph_t& graph,
  const std::string& fileName)
{
  std::cout << "save " << fileName << std::endl;

  myVertexWriter vw(graph);
  myEdgeWriter ew(graph);

  // std::map<vertex_t, size_t> vertexID;
  // boost::associative_property_map< std::map<vertex_t, size_t> > propMapVertexID(vertexID);

  // size_t i = 0;
  // for (auto vp = vertices(graph); vp.first != vp.second; ++vp.first, ++i) {
  //   const vertex_t& id = *vp.first;
  //   boost::put(propMapVertexID, id, i);
  // }

  std::ofstream dotFile(fileName);
  // this crashes in v-rep...
  boost::write_graphviz(dotFile, graph, vw, ew);//, boost::default_writer(), propMapVertexID);
}

// fig4 of Ball94

uint32_t ComputeIncrementDir(edge_t e, edge_t f, graph_t& graph, bool eValid)
{
  if (!eValid) {
    return 1;
  }
  else if (source(e, graph) == target(f, graph)
        || target(e, graph) == source(f, graph)) {
    return 1;
  } else {
    return -1;
  }
}

void ComputeIncrementDFS(uint32_t events, vertex_t v, edge_t e, graph_t& graph, bool eValid) {
  for (auto ve = in_edges(v, graph); ve.first != ve.second; ++ve.first) {
    edge_t f = *ve.first;
    if (!graph[f].chord && (!eValid || f != e)) {
      ComputeIncrementDFS(
        ComputeIncrementDir(e, f, graph, eValid) * events + graph[f].val,
        boost::source(f, graph),
        f,
        graph,
        true);
    }
  }
  for (auto ve = out_edges(v, graph); ve.first != ve.second; ++ve.first) {
    edge_t f = *ve.first;
    if (!graph[f].chord && (!eValid || f != e)) {
      ComputeIncrementDFS(
        ComputeIncrementDir(e, f, graph, eValid) * events + graph[f].val,
        boost::target(f, graph),
        f,
        graph,
        true);
    }
  }
  for (auto ve = in_edges(v, graph); ve.first != ve.second; ++ve.first) {
    edge_t f = *ve.first;
    if (graph[f].chord) {
      graph[f].increment += ComputeIncrementDir(e, f, graph, eValid) * events;
    }
  }
  for (auto ve = out_edges(v, graph); ve.first != ve.second; ++ve.first) {
    edge_t f = *ve.first;
    if (graph[f].chord) {
      graph[f].increment += ComputeIncrementDir(e, f, graph, eValid) * events;
    }
  }
}

void ComputeIncrement(vertex_t root, graph_t& graph)
{
  ComputeIncrementDFS(0, root, *edges(graph).first, graph, false);
  for (auto vp = edges(graph); vp.first != vp.second; ++vp.first) {
    const edge_t& e = *vp.first;
    if (graph[e].chord) {
      graph[e].increment += graph[e].val;
    }
  }
}

// reconstruct path (section 3.5)

void computePath(
  uint32_t val,
  graph_t& graph,
  vertex_t entry,
  std::vector<edge_t>& result)
{
  vertex_t v = entry;
  result.clear();
  uint32_t R = val;
  while (true) {
    // errs() << line(v, graph) << "\n";
    uint32_t bestVal = 0;
    vertex_t bestW;
    edge_t bestE;
    bool foundNext = false;
    for (auto ve = out_edges(v, graph); ve.first != ve.second; ++ve.first) {
      edge_t e = *ve.first;
      uint32_t val = graph[e].val;
      if (val <= R) {
        if (val >= bestVal) {
          bestVal = val;
          bestW = target(e, graph);
          bestE = e;
          foundNext = true;
        }
      }
    }
    if (!foundNext) {
      break;
    }
    v = bestW;
    R = R - bestVal;
    result.push_back(bestE);
  }
}

/////////////////////

namespace {
  struct BallLarusPass : public FunctionPass {
    static char ID;

    BallLarusPass() : FunctionPass(ID)
    {
    }

    virtual bool runOnFunction(Function &F) {
      bool changedCode = false;

      LLVMContext& Ctx = F.getContext();

      std::string name = demangle(F.getName());
      if (name == "main") {
        errs() << "Found main!\n";

        // create local variable and initialize with 0
        Instruction* insertPt = F.getEntryBlock().getFirstNonPHI();
        IRBuilder<> builder(insertPt);
        AllocaInst* counter = builder.CreateAlloca(Type::getInt64Ty(F.getContext()), nullptr, "counter");
        builder.CreateStore(ConstantInt::get(Type::getInt64Ty(F.getContext()), 0), counter);


        // Create DAG and add numPaths + val (sections 3.2 in paper)
        graph_t graph;
        vertex_t entry, exit;

        // Traverse SCCs to add vertices
        std::map<BasicBlock*, vertex_t> basicBlockToVertexMap;
        for (scc_iterator<Function*> I = scc_begin(&F),
                                      IE = scc_end(&F);
                                      I != IE; ++I) {
          // Obtain the vector of BBs in this SCC
          const std::vector<BasicBlock*>& SCCBBs = *I;
          auto v = add_vertex(graph);
          entry = v; // last one will be entry
          for (BasicBlock* BB : SCCBBs) {
            graph[v].blocks.push_back(BB);
            basicBlockToVertexMap[BB] = v;
          }
          // add edges
          uint32_t numEdges = 0;
          for (BasicBlock* BB : SCCBBs) {
            TerminatorInst *TInst = BB->getTerminator();
            for (unsigned I = 0, NSucc = TInst->getNumSuccessors(); I < NSucc; ++I) {
              BasicBlock *Succ = TInst->getSuccessor(I);
              auto vertexIter = basicBlockToVertexMap.find(Succ);
              if (vertexIter != basicBlockToVertexMap.end()) {
                vertex_t w = vertexIter->second;
                if (w != v) {
                  auto e = add_edge(v, w, graph).first;
                  graph[e].val = graph[v].numPaths;
                  graph[e].terminatorInst = TInst;
                  graph[e].terminatorIdx = I;
                  // if (TInst->getNumSuccessors() == 1) {
                  //   graph[e].insertPt = TInst;
                  // } else {
                  //   graph[e].insertPt = Succ->getFirstNonPHI();
                  // }
                  graph[v].numPaths += graph[w].numPaths;
                  ++numEdges;
                }
              } else {
                errs() << "Internal error!\n";
              }
            }
          }

          //
          if (numEdges == 0) {
            // leaf node
            graph[v].numPaths = 1;
            exit = v; // first one is exit
          }

        }

        // add fake exit -> entry
        auto e = add_edge(exit, entry, graph).first;
        graph[e].insertPt = graph[exit].blocks.at(0)->getFirstNonPHI();

        // find maximum spanning tree
        // Use -1 for all edge weights as we don't have any profiling information about "hot" edges
        // negative to use maximum (rather than minimum) spanning tree
        std::map<edge_t, int32_t> edgeWeight;
        boost::associative_property_map< std::map<edge_t, int32_t> > propMapEdgeWeight(edgeWeight);
        size_t i = 0;
        for (auto vp = edges(graph); vp.first != vp.second; ++vp.first, ++i) {
          const edge_t& e = *vp.first;
          boost::put(propMapEdgeWeight, e, -1);
          graph[e].chord = true;
        }

        std::vector<edge_t> spanningTree;
        kruskal_minimum_spanning_tree(graph, std::back_inserter(spanningTree), weight_map(propMapEdgeWeight));//propMapEdgeWeight);
        for (auto& e : spanningTree) {
          graph[e].chord = false;
        }

        // compute increment
        ComputeIncrement(entry, graph);


        saveDotFile(graph, "main.dot");


        // patch all chords
        for (auto vp = edges(graph); vp.first != vp.second; ++vp.first, ++i) {
          const edge_t& e = *vp.first;
          if (graph[e].chord) {
            if (graph[e].terminatorInst) {
              BasicBlock* bb = BasicBlock::Create(Ctx);
              F.getBasicBlockList().push_back(bb);
              IRBuilder<> builder(bb);

              Value* oldCount = builder.CreateLoad(counter, "oldCount");
              Value* newCount = builder.CreateBinOp(
                Instruction::Add,
                oldCount,
                ConstantInt::get(Type::getInt64Ty(F.getContext()), graph[e].increment),
                "newCount");
              builder.CreateStore(newCount, counter);



              BasicBlock* oldSucc = graph[e].terminatorInst->getSuccessor(graph[e].terminatorIdx);
              builder.CreateBr(oldSucc);

              graph[e].terminatorInst->setSuccessor(graph[e].terminatorIdx, bb);
            } else {
              // Load, increment and store the value back
              IRBuilder<> builder(graph[e].insertPt);
              Value* oldCount = builder.CreateLoad(counter, "oldCount");
              Value* newCount = builder.CreateBinOp(
                Instruction::Add,
                oldCount,
                ConstantInt::get(Type::getInt64Ty(F.getContext()), graph[e].increment),
                "newCount");
              builder.CreateStore(newCount, counter);
            }
          }
        }

        // add call to runtime

        Constant* printResultFunc = F.getParent()->getOrInsertFunction(
          "printResult", Type::getVoidTy(Ctx), Type::getInt64Ty(Ctx), NULL);

        IRBuilder<> builder2(graph[exit].blocks.at(0)->getTerminator());
        Value* count = builder2.CreateLoad(counter, "count");
        Value* args[] = {count};
        builder2.CreateCall(printResultFunc, args);

        dumpToFile(&F, "main.ll");
        changedCode = true;

        // testing reconstruction
        remove_edge(exit, entry, graph);

        std::vector<edge_t> result;
        errs() << "result for 0\n";
        computePath(0, graph, entry, result);
        for (auto& e : result) {
          vertex_t v = source(e, graph);
          errs() << line(v, graph) << "\n";
        }
        errs() << line(target(result.back(), graph), graph) << "\n";
        errs() << "result for 1\n";
        computePath(1, graph, entry, result);
        for (auto& e : result) {
          vertex_t v = source(e, graph);
          errs() << line(v, graph) << "\n";
        }
        errs() << line(target(result.back(), graph), graph) << "\n";
      }
      return changedCode;
    }
  };
}

char BallLarusPass::ID = 0;

// Automatically enable the pass.
// http://adriansampson.net/blog/clangpass.html
static void registerBallLarusPass(const PassManagerBuilder &,
                         legacy::PassManagerBase &PM) {
  PM.add(new BallLarusPass());
}
static RegisterStandardPasses
  RegisterMyPass(PassManagerBuilder::EP_EarlyAsPossible,
                 registerBallLarusPass);
