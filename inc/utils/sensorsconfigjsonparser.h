/**
 * @file sensorsconfigjsonparser.h
 * @author Simone Comari
 * @date 17 Jul 2019
 * @brief This file include a parser for JSON configuration file for GRAB CDPR sensors.
 */

#ifndef CABLE_ROBOT_ROBOTCONFIGJSONPARSER_H
#define CABLE_ROBOT_ROBOTCONFIGJSONPARSER_H

#include <fstream>

#include <QString>

#include "json.hpp"
#include "homogeneous_transf.h"

#include "utils/cameraparamsjsonparser.h"
#include "utils/types.h"

using json = nlohmann::json; /**< JSON library support alias */

/**
 * @brief A parser for JSON configuration file for GRAB CDPR.
 */
class SensorsConfigJsonParser
{
 public:
  /**
   * @brief RobotConfigJsonParser default constructor.
   */
  SensorsConfigJsonParser();

  /**
   * @brief Parse a JSON configuration file.
   * @param[in] filename Configuration filepath.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool ParseFile(const std::string& filename, const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file.
   * @param[in] filename Configuration filepath.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool ParseFile(const char* filename, const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file.
   * @param[in] filename Configuration filepath.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool ParseFile(const QString& filename, const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file and fills a parameters structure.
   * @param[in] filename Configuration filepath.
   * @param[out] params Parameters structure to be filled with parsed data.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool ParseFile(const std::string& filename, SensorsParams* const params,
                 const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file and fills a parameters structure.
   * @param[in] filename Configuration filepath.
   * @param[out] params Parameters structure to be filled with parsed data.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool ParseFile(const char* filename, SensorsParams* const params,
                 const bool verbose = false);
  /**
   * @brief Parse a JSON configuration file and fills a parameters structure.
   * @param[in] filename Configuration filepath.
   * @param[out] params Parameters structure to be filled with parsed data.
   * @param[in] verbose If _true_, prints content of the parsed file.
   * @return _True_ if file was correctly parsed, _false_ otherwise.
   */
  bool ParseFile(const QString& filename, SensorsParams* const params,
                 const bool verbose = false);

  /**
   * @brief Get parsed configuration structure.
   * @return A configuration structure.
   * @warning If file was not correctly parsed yet, it returns an empty structure without
   * errors or warnings.
   */
  SensorsParams GetConfigStruct() const { return config_params_; }
  /**
   * @brief Get parsed configuration structure.
   * @param[out] params The configuration structure to be filled with parsed data.
   * @warning If file was not correctly parsed yet, it returns an empty structure without
   * errors or warnings.
   */
  void GetConfigStruct(SensorsParams* const params) const { *params = config_params_; }

  /**
   * @brief Print parsed configuration parameters set, if present.
   */
  void PrintConfig() const;

 private:
  SensorsParams config_params_;
  bool file_parsed_ = false;

  bool ExtractConfig(const json& raw_data);
  bool ExtractVisionConfig(const json& raw_data);

  bool IsVisionConfigValid() const;
};

#endif // CABLE_ROBOT_ROBOTCONFIGJSONPARSER_H
