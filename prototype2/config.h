//
// Created by phi on 03/07/18.
//

#ifndef PROTOTYPE2_CONFIG_H
#define PROTOTYPE2_CONFIG_H

#include "util.h"
#include "pipeline.h"

namespace prototype2 {

class Config {
public:
  typedef boost::shared_ptr<Config> Ptr;
  Config(int argc, char **argv);
  void Open();
  void Open(std::string file);
  void Load(Pipeline &pipeline);
  void Load(Dataset &dataset, std::string sequence);
  void Close();
private:
  cv::FileStorage fs_;
  std::string file_;
};

} // namespace prototype2

#endif //PROTOTYPE2_CONFIG_H
