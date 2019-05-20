#ifndef CAMERAPARAMSJSONPARSER_H
#define CAMERAPARAMSJSONPARSER_H

#include "json.hpp"
#include "types.h"
#include <QString>
#include <fstream>
#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <stdio.h>

using json = nlohmann::json;

class CameraParamsJsonParser
{
 public:
  CameraParamsJsonParser();

  CameraParamsJsonParser(const std::string& io_filepath);

  CameraParams getCameraParams() const;

  void setCameraParams(const CameraParams& params_2_set);

  void writeJson(const CameraParams& params_in,
                 const std::string& o_filepath = SRCDIR "/output_calib.json");

  void writeJson(const std::string& o_filepath = SRCDIR "/output_calib.json");

  CameraParams decodeJson(const std::string& i_filepath = SRCDIR "/output_calib.json");

  bool decodeJson(CameraParams& params_in,
                  const std::string& i_filepath = SRCDIR "/output_calib.json");

  bool is_empty(std::ifstream& p_file);

 private:
  CameraParams params_;
};

#endif // CAMERAPARAMSJSONPARSER_H
