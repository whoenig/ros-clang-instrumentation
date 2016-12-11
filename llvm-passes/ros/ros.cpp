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

#include "yaml-cpp/yaml.h"

using namespace llvm;
using namespace std;

using namespace boost;

// helper functions

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

// Ball-Larus functions

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

std::string lineInfo(vertex_t v, const graph_t& graph) {
  for (const auto& I : *graph[v].blocks.at(0)) {
    if (const DILocation *loc = I.getDebugLoc()) {
      unsigned line = loc->getLine();
      StringRef file = loc->getFilename();
      StringRef dir = loc->getDirectory();

      std::stringstream sstr;
      sstr << file.str() << ":" << line;

      return sstr.str();
    }
  }
  return "";
}

// std::string file(vertex_t v, const graph_t& graph) {
//   for (const auto& I : *graph[v].blocks.at(0)) {
//     if (const DILocation *loc = I.getDebugLoc()) {
//       unsigned line = loc->getLine();
//       StringRef file = loc->getFilename();
//       StringRef dir = loc->getDirectory();
//       return file;
//     }
//   }
//   return "";
// }

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

// Applies path profiling following the Ball Larus approach
// The list of possible path is written out to the file
// the local counter is returned for further use
void ballLarusPatch(
  Function &F,
  const std::string& pathListFileName) {

  // create local variable and initialize with 0
  GlobalVariable* counter = new GlobalVariable(
    *F.getParent(),
    Type::getInt64Ty(F.getContext()),
    /*isConstant*/ false,
    GlobalValue::CommonLinkage,
    ConstantInt::get(Type::getInt64Ty(F.getContext()), 0),
    "ballLarusCounter");

  // return;

  Instruction* insertPt = F.getEntryBlock().getFirstNonPHI();
  IRBuilder<> builder(insertPt);
  // AllocaInst* counter = builder.CreateAlloca(Type::getInt64Ty(F.getContext()), nullptr, "counter");
  builder.CreateStore(ConstantInt::get(Type::getInt64Ty(F.getContext()), 0), counter);

  // Create DAG and add numPaths + val (sections 3.2 in paper)
  graph_t graph;
  vertex_t entry;
  std::set<vertex_t> exits;
  // bool foundExit = false;

  // Traverse SCCs to add vertices
  std::map<BasicBlock*, vertex_t> basicBlockToVertexMap;
  for (scc_iterator<Function*> I = scc_begin(&F),
                                IE = scc_end(&F);
                                I != IE; ++I) {
    // Obtain the vector of BBs in this SCC
    const std::vector<BasicBlock*>& SCCBBs = *I;

    bool isNotOnlyLandingPad = false;
    for (BasicBlock* BB : SCCBBs) {
      if (!isa<LandingPadInst>(BB->front())) {
        isNotOnlyLandingPad = true;
        break;
      }
    }
    if (!isNotOnlyLandingPad) {
      continue;
    }

    auto v = add_vertex(graph);
    entry = v; // last one will be entry
    for (BasicBlock* BB : SCCBBs) {
      if (isa<LandingPadInst>(BB->front())) {
        continue;
      }
      graph[v].blocks.push_back(BB);
      basicBlockToVertexMap[BB] = v;
    }
    // add edges
    uint32_t numEdges = 0;
    for (BasicBlock* BB : SCCBBs) {
      if (isa<LandingPadInst>(BB->front())) {
        continue;
      }

      TerminatorInst *TInst = BB->getTerminator();
      for (unsigned I = 0, NSucc = TInst->getNumSuccessors(); I < NSucc; ++I) {
        BasicBlock *Succ = TInst->getSuccessor(I);
        auto vertexIter = basicBlockToVertexMap.find(Succ);
        if (vertexIter != basicBlockToVertexMap.end()) {
          vertex_t w = vertexIter->second;
          if (w != v
              && !edge(v, w, graph).second) {
            // InvokeInst* op = dyn_cast<InvokeInst>(TInst);
            // if (op && op->getUnwindDest() == Succ) {
            //   // we can't patch unwind's, so skip this
            // } else {

              auto e = add_edge(v, w, graph).first;
              graph[e].val = graph[v].numPaths;
              graph[e].terminatorInst = TInst;
              graph[e].terminatorIdx = I;
              // if (TInst->getNumSuccessors() == 1) {
              //   graph[e].insertPt = TInst;
              // } else {
              //   graph[e].insertPt = &Succ->front();//getFirstNonPHI();
              // }
              graph[v].numPaths += graph[w].numPaths;
              ++numEdges;
            // }
          }
        }
      }
    }

    //
    if (numEdges == 0) {
      // leaf node
      graph[v].numPaths = 1;
      exits.insert(v); // first one is exit
      // foundExit = true;
    }

  }

  // add fake exit -> entry
  for (auto exit : exits) {
    auto e = add_edge(exit, entry, graph).first;
    graph[e].insertPt = &graph[exit].blocks.at(0)->front();//getFirstNonPHI();
  }

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


  saveDotFile(graph, "ballLarus.dot");


  // patch all chords
  LLVMContext& Ctx = F.getContext();
  // int idx = 0;
  for (auto vp = edges(graph); vp.first != vp.second; ++vp.first, ++i) {
    const edge_t& e = *vp.first;
    if (graph[e].chord) {
      // Load, increment and store the value back
      if (graph[e].terminatorInst) {
        // if (idx > 2)
          // continue;
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
        // ++idx;
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
  // LLVMContext& Ctx = F.getContext();
  // Constant* printResultFunc = F.getParent()->getOrInsertFunction(
  //   "printResult", Type::getVoidTy(Ctx), Type::getInt64Ty(Ctx), NULL);

  // IRBuilder<> builder2(graph[exit].blocks.at(0)->getTerminator());
  // Value* count = builder2.CreateLoad(counter, "count");
  // Value* args[] = {count};
  // builder2.CreateCall(printResultFunc, args);

  // dumpToFile(&F, "main.ll");
  // changedCode = true;

  // write file with possible paths
  // format: <path#>,line1,line2,line3,...
  for (auto exit : exits) {
    remove_edge(exit, entry, graph);
  }
  uint32_t numPaths = graph[entry].numPaths;
  std::ofstream pathListFile(pathListFileName);
  for (uint32_t i = 0; i < numPaths; ++i) {
    pathListFile << i << ",";

    std::vector<edge_t> result;
    computePath(i, graph, entry, result);
    for (auto& e : result) {
      vertex_t v = source(e, graph);
      pathListFile << lineInfo(v, graph) << ",";
    }
    pathListFile << lineInfo(target(result.back(), graph), graph) << "\n";
  }

  // return counter;
}

/////////////////////




struct PatchPoint
{
public:
  enum Type {
    TypePublisher,
    TypeSubscriber,
  };

  PatchPoint(
    const std::string& file,
    int line,
    const std::string& function,
    uint32_t topicIdx,
    uint32_t nodeIdx,
    Type type)
    : file(file)
    , line(line)
    , function(function)
    , topicIdx(topicIdx)
    , nodeIdx(nodeIdx)
    , type(type)
  {
  }

  std::string file;
  int line;
  std::string function;
  uint32_t topicIdx;
  uint32_t nodeIdx;
  Type type;
};


namespace {
  struct ROSPass : public FunctionPass {
    static char ID;

    std::vector<PatchPoint> m_patchPoints;

    ROSPass() : FunctionPass(ID)
    {
      if(const char* configFile = std::getenv("ROS_INSTRUMENTATION_CONFIG_FILE"))
      {
        try
        {
          YAML::Node config = YAML::LoadFile(configFile);
          uint32_t topicIdx = 0;
          for (const auto& topic : config["topics"]) {
            uint32_t nodeIdx = 0;
            for (const auto& node : topic["nodes"]) {
              string typeString = node["type"].as<string>();
              PatchPoint::Type type = PatchPoint::TypeSubscriber;
              if (typeString == "publisher") {
                type = PatchPoint::TypePublisher;
              }
              m_patchPoints.push_back(
                PatchPoint(
                  node["file"] ? node["file"].as<string>() : "",
                  node["line"] ? node["line"].as<int>() : -1,
                  node["function"] ? node["function"].as<string>() : "",
                  topicIdx,
                  nodeIdx,
                  type
                  ));
              ++nodeIdx;
            }
            ++topicIdx;
          }
        }
        catch (std::exception& e)
        {
          errs() << "Error while parsing config file:\n";
          errs() << e.what() << "\n";
        }
      } else {
        errs() << "No config file specified!\n";
      }
    }

    // http://docs.ros.org/kinetic/api/roscpp_serialization/html/serialization_8h_source.html#l00845
/*
    844 template<typename M>
    845 inline SerializedMessage serializeMessage(const M& message)
    846 {
    847   SerializedMessage m;
    848   uint32_t len = serializationLength(message);
    849   m.num_bytes = len + 4; // <= change to + 12
    850   m.buf.reset(new uint8_t[m.num_bytes]);
    851
    852   OStream s(m.buf.get(), (uint32_t)m.num_bytes);
    853   serialize(s, (uint32_t)m.num_bytes - 4); // <= change to -12
    854   m.message_start = s.getData();
    855   serialize(s, message);
          serialize(s, timestamp()); // <= new call
    856
    857   return m;
    858 }
*/

    void patchserializeMessageFunction(Function* f)
    {
      dumpToFile(f, "serializeMessageOriginal.ll");
      // errs() << "before patching\n";
      // f->dump();
      // Get the function to call from our runtime library.
      LLVMContext &Ctx = f->getContext();
      Constant *timestamp = f->getParent()->getOrInsertFunction(
        "timestamp", IntegerType::get(Ctx, 64), NULL
      );
      // Function* rtlibFunc1 = f->getParent()->getFunction("rtlibFunc1");

      // errs() << rtlibFunc1;
      // return;
      // int b = 0;
      int numSerializeCalls = 0;
      Function* firstCalledFunc;
      for (auto &B : *f) {
        // if (b == 0) {
        for (auto &I : B) {
          if (auto* op = dyn_cast<BinaryOperator>(&I)) {
            // op->dump();
            Value* rhs = op->getOperand(1);
            ConstantInt* rhsCI = dyn_cast<ConstantInt>(rhs);
            assert(rhsCI);
            assert(rhsCI->equalsInt(4));
            // rhs->dump();
            // rhs->getType()->dump();
            op->setOperand(1, ConstantInt::get(IntegerType::get(Ctx, 32), 8));
            // op->dump();
          }
          if (auto *op = dyn_cast<InvokeInst>(&I)) {
            auto calledFunc = op->getCalledFunction();
            if (calledFunc->getName().find("serialization9serialize") != std::string::npos) {
              ++numSerializeCalls;
              if (numSerializeCalls == 1) {
                firstCalledFunc = calledFunc;
              }
              // if (numSerializeCalls == 1) {
              //   BasicBlock* bb = BasicBlock::Create(Ctx);
              //   f->getBasicBlockList().push_back(bb);
              //   // insert after op
              //   IRBuilder<> builder(bb);
              //   // builder.SetInsertPoint(&B, ++builder.GetInsertPoint());

              //   // Insert a call "serialize(s, timestamp());"
              //   Value* args[] = {op->getArgOperand(0), ConstantInt::get(IntegerType::get(Ctx, 32), 3)};//timestamp};
              //   builder.CreateInvoke(firstCalledFunc, op->getNormalDest(), op->getUnwindDest(), args);
              //   op->setNormalDest(bb);

              // }
            }

          }
          //   // I.dump();
          //   // Insert *after* `op`.
          //   IRBuilder<> builder(&I);
          //   // builder.SetInsertPoint(&B, ++builder.GetInsertPoint());

          //   // Insert a call to our function.
          //   // Value* args[] = {};
          //   builder.CreateCall(rtlibFunc1);


          //   errs() << "patch3\n";
          //   // errs() << "after patching\n";
          //   // f->dump();
          //   dumpToFile(f, "serializeMessagePatched.ll");
          //   return;
          //   // return true;
          // // }
        // }
      }
      // ++b;
      }

      dumpToFile(f, "serializeMessagePatched.ll");

    }

    void patchPublishFunction(
      Function* f,
      uint32_t topicIdx,
      uint32_t nodeIdx)
    {
      // for (auto &B : *f) {
      //   for (auto &I : B) {
      //     // I.dump();
      //     if (auto* op = dyn_cast<InvokeInst>(&I)) {

      //       auto calledFunc = op->getCalledFunction();
      //       // errs() << calledFunc->getName() << "\n";

      //       if (calledFunc->getName().find("boost4bind") != std::string::npos) {
      //         errs() << demangle(calledFunc->getName()) << "\n";
      //       // if (calledFunc->getName().find("Publish") != std::string::npos) {
      //         //clone function (to create an instrumented version)
      //         // I.dump();
      //         Value* v1 = op->getArgOperand(0);
      //         Function* f2 = dyn_cast<Function>(v1);
      //         assert(f2);
      //         errs() << demangle(f2->getName()) << "\n";
      //         assert(f2->getName().find("serializeMessage") != std::string::npos);
      //         ValueToValueMapTy VMap;
      //         Function* clonedFunc = CloneFunction(f2, VMap, false);
      //         f->getParent()->getFunctionList().push_back(clonedFunc);
      //         // instrument clone
      //         patchserializeMessageFunction(clonedFunc);
      //         // set call target to clone
      //         op->setArgOperand(0, clonedFunc);
      //         errs() << "patched2!\n";
      //         return;
      //       }
      //     }
      //   }
      // }
      dumpToFile(f, "publishOriginal.ll");

      LLVMContext &Ctx = f->getContext();
      Constant *addToLog = f->getParent()->getOrInsertFunction(
        "addToLog", Type::getVoidTy(Ctx), f->getFunctionType()->getParamType(0), IntegerType::get(Ctx, 32), IntegerType::get(Ctx, 32), NULL
      );

      for (auto &B : *f) {
        for (auto &I : B) {
          IRBuilder<> builder(&I);
          // Insert a call to our function.
          GlobalVariable* counter = f->getParent()->getGlobalVariable("ballLarusCounter");
          Value* count = builder.CreateLoad(counter, "count");
          Value* args[] = {
            &(*(f->getArgumentList().begin())),
            ConstantInt::get(IntegerType::get(Ctx, 32), topicIdx),
            ConstantInt::get(IntegerType::get(Ctx, 32), nodeIdx),
            count
          };
          builder.CreateCall(addToLog, args);
          dumpToFile(f, "publishPatched.ll");
          return;
        }
      }
    }

    void patchSubscribeCallback(
      Function* f,
      uint32_t topicIdx,
      uint32_t nodeIdx)
    {
      dumpToFile(f, "subscribeOriginal.ll");

      LLVMContext &Ctx = f->getContext();
      Constant *addToLog = f->getParent()->getOrInsertFunction(
        "addToLogSubscriber", Type::getVoidTy(Ctx), IntegerType::get(Ctx, 32), IntegerType::get(Ctx, 32), NULL
      );

      for (auto &B : *f) {
        for (auto &I : B) {
          IRBuilder<> builder(&I);
          // Insert a call to our function.
          Value* args[] = {
            ConstantInt::get(IntegerType::get(Ctx, 32), topicIdx),
            ConstantInt::get(IntegerType::get(Ctx, 32), nodeIdx)};
          builder.CreateCall(addToLog, args);
          dumpToFile(f, "subscribePatched.ll");
          return;
        }
      }
    }

    void patchMain(
      Function* f)
    {
      // dumpToFile(f, "mainOriginal.ll");

      uint32_t numTopics = 0;
      uint32_t maxNumNodes = 0;
      for (const auto& pp : m_patchPoints) {
        numTopics = std::max(numTopics, pp.topicIdx + 1);
        maxNumNodes = std::max(maxNumNodes, pp.nodeIdx + 1);
      }

      LLVMContext &Ctx = f->getContext();
      Constant *initPatch = f->getParent()->getOrInsertFunction(
        "initPatch", Type::getVoidTy(Ctx), IntegerType::get(Ctx, 32), IntegerType::get(Ctx, 32), NULL
      );

      for (auto &B : *f) {
        for (auto &I : B) {
          IRBuilder<> builder(&I);
          // Insert a call to our function.
          Value* args[] = {
            ConstantInt::get(IntegerType::get(Ctx, 32), numTopics),
            ConstantInt::get(IntegerType::get(Ctx, 32), maxNumNodes)};
          builder.CreateCall(initPatch, args);
          // dumpToFile(f, "mainPatched.ll");
          return;
        }
      }
    }

    const PatchPoint* shouldPatchPublisher(
      StringRef file,
      unsigned line,
      const std::string& functionName) const {
      for (const auto& pp : m_patchPoints) {
        if (pp.type == PatchPoint::TypePublisher
         && file.str().find(pp.file) != string::npos
         && functionName.find(pp.function) != string::npos
         && ((line == pp.line) || (pp.line == -1))) {
          return &pp;
        }
      }
      return nullptr;
    }

    const PatchPoint* shouldPatchSubscriber(const std::string& name) const {
      for (const auto& pp : m_patchPoints) {
        if (pp.type == PatchPoint::TypeSubscriber
         && pp.function.size() > 0 && name.find(pp.function) != string::npos) {
          return &pp;
        }
      }
      return nullptr;
    }


    virtual bool runOnFunction(Function &F) {
      bool changedCode = false;

      // patch callback function if desired
      std::string name = demangle(F.getName());
      if (name == "main"
          && m_patchPoints.size() > 0) {
        errs() << "Patch main...\n";
        patchMain(&F);
        changedCode = true;
      }

      if (const PatchPoint* pp = shouldPatchSubscriber(name)) {
        errs() << "Found Subscriber: " << name << "\n";
        patchSubscribeCallback(&F, pp->topicIdx, pp->nodeIdx);
        errs() << "...patched!\n";

        stringstream sstr;
        sstr << "paths_" << pp->topicIdx << "_" << pp->nodeIdx << ".txt";
        std::ofstream paths(sstr.str());
        paths << name << "\n";

        changedCode = true;
      }

      // iterate over instructions and patch desired ros::Publisher::publish calls
      for (auto& B : F) {
        for (auto& I : B) {
          Function* calledFunc = nullptr;
          if (auto* op = dyn_cast<InvokeInst>(&I)) {
            calledFunc = op->getCalledFunction();
          }
          if (auto* op = dyn_cast<CallInst>(&I)) {
            calledFunc = op->getCalledFunction();
          }
          if (calledFunc) {
            std::string calledName = demangle(calledFunc->getName());
            if (calledName.find("ros::Publisher::publish") != std::string::npos) {
              // errs() << calledFunc->getName() << "\n";
              if (DILocation *loc = I.getDebugLoc()) {
                unsigned line = loc->getLine();
                StringRef file = loc->getFilename();
                StringRef dir = loc->getDirectory();
                errs() << "Found ros::Publisher::publish at " << dir << " " << file << " " << line << "\n";
                if (const PatchPoint* pp = shouldPatchPublisher(file, line, name)) {
                  // patch this function
                  dumpToFile(&F, "publishCallerOriginal.ll");

                  stringstream sstr;
                  sstr << "paths_" << pp->topicIdx << "_" << pp->nodeIdx << ".txt";
                  ballLarusPatch(F, sstr.str());

                  ValueToValueMapTy VMap;
                  Function* clonedFunc = CloneFunction(calledFunc, VMap, false);
                  F.getParent()->getFunctionList().push_back(clonedFunc);
                  // instrument clone
                  patchPublishFunction(clonedFunc, pp->topicIdx, pp->nodeIdx);
                  // set call target to clone
                  if (auto* op = dyn_cast<InvokeInst>(&I)) {
                    op->setCalledFunction(clonedFunc);
                  }
                  if (auto* op = dyn_cast<CallInst>(&I)) {
                    op->setCalledFunction(clonedFunc);
                  }
                  changedCode = true;
                  dumpToFile(&F, "publishCallerPatched.ll");
                  errs() << "...patched!\n";
                } else {
                  errs() << "...ignored!\n";
                }
              } else {
                errs() << "Found ros::Publisher::publish with no source information.\n";
                errs() << "...ignored!\n";
              }
            }
          }
        }
      }
      return changedCode;

      // // errs() << F.getParent()->getName() << "\n";

      // // errs() << demangle(F.getName()) << "\n";
      // // if (F.getName().find("serializeMessage") != std::string::npos) {
      // //   patchserializeMessageFunction(&F);
      // //   return true;
      // // }
      // // if (F.getName().find("boost4bind")!= std::string::npos) {
      // //   errs() << "found boostbind" << "\n";
      // //   dumpToFile(&F, "boostbind.ll");
      // //   patchserializeMessageFunction(&F);
      // //   return true;
      // // }
      // // if (F.getName().find("publish")!= std::string::npos) {
      // //   errs() << "found publish" << "\n";
      // //   dumpToFile(&F, "publish.ll");
      // //   // patchserializeMessageFunction(&F);
      // //   return true;
      // // }

      // // return changedCode;


      //   // Function* rtlibFunc1 = F.getParent()->getFunction("rtlibFunc1");

      //   // errs() << rtlibFunc1;

      // // if (F.getName().find("rtlib") != std::string::npos) {
      // //   errs() << F.getName() << &F << "\n";


      // // }

      // if (F.getName() == "main") {
      //   errs() << "found main!\n";

      //   // errs() << "Function body:\n";
      //   // F.dump();

      //   for (auto& B : F) {
      //     // errs() << "Basic block:\n";
      //     // B.dump();
      //     for (auto& I : B) {
      //       if (auto* op = dyn_cast<InvokeInst>(&I)) {
      //         auto calledFunc = op->getCalledFunction();
      //         if (calledFunc->getName().find("ros9Publisher7publish") != std::string::npos) {
      //           if (DILocation *loc = I.getDebugLoc()) {
      //             unsigned line = loc->getLine();
      //             StringRef file = loc->getFilename();
      //             StringRef dir = loc->getDirectory();
      //             errs() << dir << " " << file << " " << line << "\n";
      //           }

      //           auto dl = I.getDebugLoc();
      //           if (dl) {
      //             errs() << "have dl\n" << dl.getLine()  << "\n";
      //           }
      //         // if (calledFunc->getName().find("Publish") != std::string::npos) {
      //           //clone function (to create an instrumented version)
      //           // I.dump();
      //           ValueToValueMapTy VMap;
      //           Function* clonedFunc = CloneFunction(calledFunc, VMap, false);
      //           F.getParent()->getFunctionList().push_back(clonedFunc);
      //           // instrument clone
      //           patchPublishFunction(clonedFunc);
      //           // patchserializeMessageFunction(clonedFunc);
      //           // set call target to clone
      //           op->setCalledFunction(clonedFunc);
      //           changedCode = true;
      //           errs() << "patched!\n";
      //         }
      //       }
      //     }
      //     // for (auto& I : B) {
      //     //   errs() << I.getOpcodeName() << "\n";
      //     //   // errs() << "Instruction: ";
      //     //   // I.dump();
      //     // }
      //   }

      // }

      // // errs() << "I saw a function called " << F.getName() << "!\n";
      // return changedCode;
      // // // Get the function to call from our runtime library.
      // // LLVMContext &Ctx = F.getContext();
      // // Constant *logFunc = F.getParent()->getOrInsertFunction(
      // //   "logop", Type::getVoidTy(Ctx), Type::getInt32Ty(Ctx), NULL
      // // );

      // // for (auto &B : F) {
      // //   for (auto &I : B) {
      // //     if (auto *op = dyn_cast<BinaryOperator>(&I)) {
      // //       // Insert *after* `op`.
      // //       IRBuilder<> builder(op);
      // //       builder.SetInsertPoint(&B, ++builder.GetInsertPoint());

      // //       // Insert a call to our function.
      // //       Value* args[] = {op};
      // //       builder.CreateCall(logFunc, args);

      // //       return true;
      // //     }
      // //   }
      // // }

      // // return false;
    }
  };
}

char ROSPass::ID = 0;

// Automatically enable the pass.
// http://adriansampson.net/blog/clangpass.html
static void registerROSPass(const PassManagerBuilder &,
                         legacy::PassManagerBase &PM) {
  PM.add(new ROSPass());
}
static RegisterStandardPasses
  RegisterMyPass(PassManagerBuilder::EP_EarlyAsPossible,
                 registerROSPass);
