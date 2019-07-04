/**
 * @file cameraparamsjsonparser.h
 * @author Giovanni Melandri
 * @date 01 Jul 2019
 * @brief Definition of a class aimed to parsing data to and from a text file
 * of "json" format. It mainly contains the costructors, one getter function,
 * one setter function, and two principal functions which have the purpose to
 * write to and read from a file.
 */

#ifndef CAMERAPARAMSJSONPARSER_H
#define CAMERAPARAMSJSONPARSER_H

#include <QString>
#include <fstream>
#include <iostream>

#include "json.hpp"
#include "types.h"

using json = nlohmann::json; /**< class contained in json.hpp library */

/**
 * @brief The CameraParamsJsonParser class
 */
class CameraParamsJsonParser
{
 public:
  /**
   * @brief CameraParamsJsonParser default constructor
   */
  CameraParamsJsonParser();
  /**
   * @brief CameraParamsJsonParser constructor
   * @param[in] io_filepath file from which data will be parsed
   */
  CameraParamsJsonParser(const std::string& io_filepath);

  /**
   * @brief getter function to obtain the private parsed camera params
   * @return params_ camera params from parser object attribute
   */
  CameraParams getCameraParams() const;


  /**
   * @brief setter function to insert external params to parse
   * @param[in] params_2_set camera parameters set as attribute
   */
  void setCameraParams(const CameraParams& params_2_set);

  /**
   * @brief "write to file" function, first version
   * @param[in] params_in external camera parameters to be parsed to file
   * @param[in] o_filepath output .json file directory
   */
  void writeJson(const CameraParams& params_in,
                 const std::string& o_filepath = SRCDIR "/output_calib.json");
  /**
   * @brief "write to file" function, second version: it parses to file the data
   * included in the params_ attribute of the parser object
   * @param[in] o_filepath output .json file directory
   */
  void writeJson(const std::string& o_filepath = SRCDIR "/output_calib.json");

  /**
   * @brief "read from file" function, first version
   * @param[in] i_filepath file from which parse the data
   * @return params_ camera params parser object attribute
   */
  CameraParams decodeJson(const std::string& i_filepath = SRCDIR "/output_calib.json");
  /**
   * @brief "read from file" function, second version: boolean function, the
   * read params are set as a non cost parameter
   * @param[out] params_in camera parameters read. not set as
   * class attribute
   * @param[in] i_filepath file from which parse the data
   * @return true/false
   */
  bool decodeJson(CameraParams& params_in,
                  const std::string& i_filepath = SRCDIR "/output_calib.json");

  /**
   * @brief auxiliar boolean function to control if a file is empty
   * @param p_file
   * @return true if the text file is empty, false otherwise
   */
  bool isEmpty(std::ifstream& p_file);

 private:
  CameraParams params_;
};

#endif // CAMERAPARAMSJSONPARSER_H
