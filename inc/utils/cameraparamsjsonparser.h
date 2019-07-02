/**
 * @file cameraparamsjsonparser.h
 * @author
 * @date 01 Jul 2019
 * @brief ...
 */

#ifndef CAMERAPARAMSJSONPARSER_H
#define CAMERAPARAMSJSONPARSER_H

#include <QString>
#include <fstream>
#include <iostream>

#include "json.hpp"
#include "types.h"

using json = nlohmann::json; /**< ... */

/**
 * @brief The CameraParamsJsonParser class
 */
class CameraParamsJsonParser
{
 public:
  /**
   * @brief CameraParamsJsonParser
   */
  CameraParamsJsonParser();
  /**
   * @brief CameraParamsJsonParser
   * @param[] io_filepath
   */
  CameraParamsJsonParser(const std::string& io_filepath);

  /**
   * @brief getCameraParams
   * @return
   */
  CameraParams getCameraParams() const;


  /**
   * @brief setCameraParams
   * @param params_2_set
   */
  void setCameraParams(const CameraParams& params_2_set);

  /**
   * @brief writeJson
   * @param params_in
   * @param o_filepath
   */
  void writeJson(const CameraParams& params_in,
                 const std::string& o_filepath = SRCDIR "/output_calib.json");
  /**
   * @brief writeJson
   * @param o_filepath
   */
  void writeJson(const std::string& o_filepath = SRCDIR "/output_calib.json");

  /**
   * @brief decodeJson
   * @param i_filepath
   * @return
   */
  CameraParams decodeJson(const std::string& i_filepath = SRCDIR "/output_calib.json");
  /**
   * @brief ...
   * @param[out] params_in ...
   * @param[in] i_filepath ...
   * @return ...
   */
  bool decodeJson(CameraParams& params_in,
                  const std::string& i_filepath = SRCDIR "/output_calib.json");

  /**
   * @brief isEmpty
   * @param p_file
   * @return
   */
  bool isEmpty(std::ifstream& p_file);

 private:
  CameraParams params_;
};

#endif // CAMERAPARAMSJSONPARSER_H
