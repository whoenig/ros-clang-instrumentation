#include <iostream>
#include <stdint.h>
#include <fstream>
#include <sstream>

#include <time.h>
#include <sys/time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include <algorithm>
#include <cstdio>

using namespace std;

static inline uint64_t timestamp(void)
{
  struct timeval time;
  gettimeofday(&time, NULL);
  uint64_t totalTime = time.tv_sec * 1e6 + time.tv_usec;
  return totalTime;
}

extern "C" void initPatch(
  uint32_t numTopics,
  uint32_t maxNumNodes)
{
  // delete all relevant files (if they exist)
  for (uint32_t topicIdx = 0; topicIdx < numTopics; ++topicIdx) {
    for (uint32_t nodeIdx = 0; nodeIdx < maxNumNodes; ++nodeIdx) {
      stringstream sstr;
      sstr << topicIdx << "_" << nodeIdx << ".txt";
      remove(sstr.str().c_str());
    }
  }
}

// TODO: add hash of message?
extern "C" void addToLog(
  ros::Publisher* pub,
  uint32_t topicIdx,
  uint32_t nodeIdx,
  int64_t counter)
{
  uint64_t t = timestamp();
  if (pub->getNumSubscribers() > 0) {
    stringstream sstr;
    sstr << topicIdx << "_" << nodeIdx << ".txt";
    ofstream out(sstr.str().c_str(), ios::app);
    out << t << "," << counter << std::endl;
  }
}

extern "C" void addToLogSubscriber(
  uint32_t topicIdx,
  uint32_t nodeIdx)
{
  uint64_t t = timestamp();

  stringstream sstr;
  sstr << topicIdx << "_" << nodeIdx << ".txt";
  ofstream out(sstr.str().c_str(), ios::app);
  out << t << std::endl;
}
