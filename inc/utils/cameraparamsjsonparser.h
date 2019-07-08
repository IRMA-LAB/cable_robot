/**
 * @file cameraparamsjsonparser.h
 * @author Giovanni Melandri, Simone Comari
 * @date 01 Jul 2019
 * @brief This file include a JSON parser for decoding/encoding camera parameters
 * according to a custom standard format.
 */

#ifndef CAMERAPARAMSJSONPARSER_H
#define CAMERAPARAMSJSONPARSER_H

#include <QString>
#include <fstream>
#include <iostream>

#include "json.hpp"
#include "types.h"

using json = nlohmann::json; /**< Alias for custom json namespace. */

/**
 * @brief This class can both read and write a JSON file including camera parameters.
 *
 * Despite the name, this object not only can parse a JSON file to extract camera
 * parameters from a file and store the in a compliant stucture, but can also perform the
 * opposite operation, that is writing a given structure onto a file.
 * @todo redefine write/read functions to avoid duplicate code..
 */
class CameraParamsJsonParser
{
 public:
  /**
   * @brief Default constructor.
   */
  CameraParamsJsonParser();
  /**
   * @brief Full constructor
   * @param[in] io_filepath Location of file from which data will be parsed.
   */
  CameraParamsJsonParser(const std::string& io_filepath);

  /**
   * @brief Return latest camera parameters parsed/loaded.
   * @return Latest camera parameters parsed.
   */
  CameraParams getCameraParams() const;

  /**
   * @brief Set camera parameters to be encoded and dumped onto a file.
   * @param[in] params_2_set Camera parameters.
   */
  void setCameraParams(const CameraParams& params_2_set);

  /**
   * @brief Write given camera parameters onto a JSON file.
   * @param[in] params_in External camera parameters to be dumped onto the file.
   * @param[in] o_filepath Output file directory.
   */
  void writeJson(const CameraParams& params_in,
                 const std::string& o_filepath = SRCDIR "/output_calib.json");
  /**
   * @brief Write current camera parameters onto a JSON file, if any.
   * @param[in] o_filepath output .json file directory
   */
  void writeJson(const std::string& o_filepath = SRCDIR "/output_calib.json");

  /**
   * @brief Read camera parameters from a file.
   * @param[in] i_filepath File from which to parse the data.
   * @return A structure containing parsed camera parameters.
   */
  CameraParams decodeJson(const std::string& i_filepath = SRCDIR "/output_calib.json");
  /**
   * @brief Read camera parameters from a file.
   * @param[out] params_in An empty structure to be filled with parsed camera parameters.
   * @param[in] i_filepath File from which to parse the data.
   * @return _True_ if parsing was successful, _False_ otherwise.
   */
  bool decodeJson(CameraParams& params_in,
                  const std::string& i_filepath = SRCDIR "/output_calib.json");

 private:
  CameraParams params_;

  bool isEmpty(std::ifstream& p_file);
};

#endif // CAMERAPARAMSJSONPARSER_H
