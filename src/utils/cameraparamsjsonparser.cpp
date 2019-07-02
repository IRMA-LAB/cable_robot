/**
 * @file cameraparamsjsonparser.cpp
 * @author ...
 * @date 01 Jul 2019
 * @brief Implementation of class decalred in cameraparamsjsonparser.h
 */

#include "utils/cameraparamsjsonparser.h"

CameraParamsJsonParser::CameraParamsJsonParser() {}

CameraParamsJsonParser::CameraParamsJsonParser(const std::string& io_filepath)
{
  decodeJson(io_filepath);
}

//--------- Public functions --------------------------------------------------------//

CameraParams CameraParamsJsonParser::getCameraParams() const { return params_; }

void CameraParamsJsonParser::setCameraParams(const CameraParams& params_2_set)
{
  params_ = params_2_set;
}

void CameraParamsJsonParser::writeJson(
  const CameraParams& params_in,
  const std::string& o_filepath /*= SRCDIR "/output_calib.json*/)
{
  std::string str = o_filepath;
  size_t found    = str.find_last_of(".");
  if (found == std::string::npos)
    str.append(".json");
  else if (str.substr(found).compare(("json")) != 0)
    str.replace(found, std::string::npos, ".json");

  std::ofstream o_file(str);
  if (!o_file.is_open())
    std::cerr << "\n\nERROR\tcould not create file\n\n";

  json parameters_2_write;

  std::vector<double> camera_matrix(params_in.camera_matrix.begin<double>(),
                                    params_in.camera_matrix.end<double>());

  for (uchar i = 0; i < camera_matrix.size(); i++)
    if (camera_matrix[i] < 0)
    {
      std::cerr << "\n\nERROR\tinvalid camera matrix, all elements must be "
                   "positive\n\n\tcannot write to file\n\n";

      return;
    }

  if (camera_matrix[1] != 0.0 || camera_matrix[3] != 0.0 || camera_matrix[6] != 0.0 ||
      camera_matrix[7] != 0.0 || camera_matrix[8] != 1.0)
  {
    std::cerr << "\n\nERROR\tinvalid camera matrix, wrong matrix "
                 "definition\n\n\tcannot write to file\n\n";

    return;
  }

  parameters_2_write["camera_matrix"] = camera_matrix;

  std::vector<double> dist_coeff(params_in.dist_coeff.begin<double>(),
                                 params_in.dist_coeff.end<double>());


  parameters_2_write["dist_coeff"] = dist_coeff;

  o_file << std::setw(2) << parameters_2_write << std::endl;

  o_file.close();
}

void CameraParamsJsonParser::writeJson(
  const std::string& o_filepath /*= SRCDIR "/output_calib.json"*/)
{
  std::string str = o_filepath;
  size_t found    = str.find_last_of(".");
  if (found == std::string::npos)
    str.append(".json");
  else if (str.substr(found).compare(("json")) != 0)
    str.replace(found, std::string::npos, ".json");

  std::ofstream o_file(str);
  if (!o_file.is_open())
    std::cerr << "\n\nERROR\tcould not create file\n\n";

  json parameters_2_write;
  std::vector<double> camera_matrix(params_.camera_matrix.begin<double>(),
                                    params_.camera_matrix.end<double>());
  for (uchar i = 0; i < camera_matrix.size(); i++)
    if (camera_matrix[i] < 0)
    {
      std::cerr << "\n\nERROR\tinvalid camera matrix, all elements must be "
                   "positive\n\n\tcannot write to file\n\n";
      return;
    }
  if (camera_matrix[1] != 0.0 || camera_matrix[3] != 0.0 || camera_matrix[6] != 0.0 ||
      camera_matrix[7] != 0.0 || camera_matrix[8] != 1.0)
  {
    std::cerr << "\n\nERROR\tinvalid camera matrix, wrong matrix "
                 "definition\n\n\tcannot write to file\n\n";
    return;
  }
  parameters_2_write["camera_matrix"] = camera_matrix;
  std::vector<double> dist_coeff(params_.dist_coeff.begin<double>(),
                                 params_.dist_coeff.end<double>());
  parameters_2_write["dist_coeff"] = dist_coeff;

  o_file << std::setw(2) << parameters_2_write << std::endl;
  o_file.close();
}

CameraParams CameraParamsJsonParser::decodeJson(
  const std::string& i_filepath /*= SRCDIR "/output_calib.json"*/)
{
  std::ifstream i_file(i_filepath);

  if (!i_file.is_open())
  {
    std::cerr << "\n\nERROR\tcould not open file\n\n";
  }

  if (isEmpty(i_file))
  {
    std::cerr << "\n\nERROR\tfound an empty file\n\n\tcamera parameters set as "
                 "default\n\n";
    return params_;
  }

  json parameters_2_read;

  i_file >> parameters_2_read;

  i_file.close();

  std::vector<double> camera_matrix =
    parameters_2_read["camera_matrix"].get<std::vector<double>>();

  for (uchar i = 0; i < camera_matrix.size(); i++)
  {
    if (camera_matrix[i] < 0)
    {
      std::cerr << "\n\nERROR\tinvalid camera matrix, all elements must be "
                   "positive\n\n";

      return params_;
    }
  }

  if (camera_matrix[1] != 0.0 || camera_matrix[3] != 0.0 || camera_matrix[6] != 0.0 ||
      camera_matrix[7] != 0.0 || camera_matrix[8] != 1.0)
  {
    std::cerr << "\n\nERROR\tinvalid camera matrix, wrong matrix definition\n\n";

    return params_;
  }

  memcpy(params_.camera_matrix.data, camera_matrix.data(),
         camera_matrix.size() * sizeof(double));

  std::vector<double> dist_coeff =
    parameters_2_read["dist_coeff"].get<std::vector<double>>();

  memcpy(params_.dist_coeff.data, dist_coeff.data(), dist_coeff.size() * sizeof(double));

  return params_;
}

bool CameraParamsJsonParser::decodeJson(
  CameraParams& params_in,
  const std::string& i_filepath /*= SRCDIR "/output_calib.json"*/)
{
  std::ifstream i_file(i_filepath);

  if (!i_file.is_open())
  {
    std::cerr << "\n\nERROR\tcould not open file\n\n";
    return false;
  }

  if (isEmpty(i_file))
  {
    std::cerr << "\n\nERROR\tfound an empty file\n\n\tcamera parameters set as "
                 "default\n\n";
    return false;
  }

  json parameters_2_read;

  i_file >> parameters_2_read;

  i_file.close();

  std::vector<double> camera_matrix =
    parameters_2_read["camera_matrix"].get<std::vector<double>>();

  for (uchar i = 0; i < camera_matrix.size(); i++)
  {
    if (camera_matrix[i] < 0)
    {
      std::cerr << "\n\nERROR\tinvalid camera matrix, all elements must be "
                   "positive\n\n";

      return false;
    }
  }

  if (camera_matrix[1] != 0.0 || camera_matrix[3] != 0.0 || camera_matrix[6] != 0.0 ||
      camera_matrix[7] != 0.0 || camera_matrix[8] != 1.0)
  {
    std::cerr << "\n\nERROR\tinvalid camera matrix, wrong matrix definition\n\n";

    return false;
  }

  memcpy(params_in.camera_matrix.data, camera_matrix.data(),
         camera_matrix.size() * sizeof(double));

  std::vector<double> dist_coeff =
    parameters_2_read["dist_coeff"].get<std::vector<double>>();

  memcpy(params_in.dist_coeff.data, dist_coeff.data(),
         dist_coeff.size() * sizeof(double));

  return true;
}

bool CameraParamsJsonParser::isEmpty(std::ifstream& p_file)
{
  return p_file.peek() == std::ifstream::traits_type::eof();
}
