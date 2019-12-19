//
// Created by junlinp on 2019-12-19.
//

#include "las.hpp"

void las::WriteHeader(std::ostream &os) {
  char file_signature[4] = {'L', 'A', 'S', 'F'};

  char unused_1[2 + 2 + 4 + 2 + 2 + 8] = {0};
  char version_major = 1;
  char version_minor = 2;
  char unused_2[32 + 32 + 2 + 2] = {0};
  uint16_t header_size = 227;
  uint32_t offset_to_point_data = 227;
  uint32_t number_of_variable_length_records = 0;
  char point_data_format_id = 2;
  uint16_t point_data_record_length = 26;
  uint32_t number_of_point_records = 123;
  char unused_3[20] = {0};
  double x_scale_factor = 1e-6;
  double y_scale_factor = 1e-6;
  double z_scale_factor = 1e-6;
  double x_offset = 0;
  double y_offset = 0;
  double z_offset = 0;
  double max_x = 0;
  double min_x = 0;
  double max_y = 0;
  double min_y = 0;
  double max_z = 0;
  double min_z = 0;

  os.write(file_signature, 4);
  os.write(unused_1, 20);
  os.write(&version_major, 1);
  os.write(&version_minor, 1);
  os.write(unused_2, 68);
  os.write((char*)&header_size, 2);
  os.write((char*)&offset_to_point_data, 4);
  os.write((char*)&number_of_variable_length_records, 4);
  os.write((char*)&point_data_format_id, 1);
  os.write((char*)&point_data_record_length, 2);
  os.write((char*)&number_of_point_records, 4);
  os.write(unused_3, 20);
  os.write((char*)&x_scale_factor, 8);
  os.write((char*)&y_scale_factor, 8);
  os.write((char*)&z_scale_factor, 8);
  os.write((char*)&x_offset, 8);
  os.write((char*)&y_offset, 8);
  os.write((char*)&z_offset, 8);
  os.write((char*)&max_x, 8);
  os.write((char*)&min_x, 8);
  os.write((char*)&max_y, 8);
  os.write((char*)&min_y, 8);
  os.write((char*)&max_z, 8);
  os.write((char*)&min_z, 8);

}
void las::ReadHeader(std::istream &is) {
  char file_signature[5] = {0};

  char unused_1[2 + 2 + 4 + 2 + 2 + 8] = {0};
  char version_major = 1;
  char version_minor = 2;
  char unused_2[32 + 32 + 2 + 2] = {0};
  uint16_t header_size = 243;
  uint32_t offset_to_point_data = 243;
  uint32_t number_of_variable_length_records = 0;
  char point_data_format_id = 2;
  uint16_t point_data_record_length = 26;
  uint32_t number_of_point_records;
  char unused_3[20] = {0};
  double x_scale_factor = 1e-6;
  double y_scale_factor = 1e-6;
  double z_scale_factor = 1e-6;

  is.read(file_signature, 4);
  is.read(unused_1, 20);
  is.read(&version_major, 1);
  is.read(&version_minor, 1);
  is.read(unused_2, 68);
  is.read((char*)&header_size, 2);
  is.read((char*)&offset_to_point_data, 4);
  is.read((char*)&number_of_variable_length_records, 4);
  is.read((char*)&point_data_format_id, 1);
  is.read((char*)&point_data_record_length, 2);
  is.read((char*)&number_of_point_records, 4);
  is.read(unused_3, 20);
  is.read((char*)&x_scale_factor, 8);
  is.read((char*)&y_scale_factor, 8);
  is.read((char*)&z_scale_factor, 8);

  std::cout << "file_signature : " << file_signature << std::endl;
  std::cout << "header_size : " << header_size << std::endl;
  std::cout << "offset_to_point_data : " << offset_to_point_data << std::endl;
  std::cout << "number_of_variable_length_records : " << number_of_variable_length_records << std::endl;
  std::cout << "point_data_record_length : " << point_data_record_length << std::endl;
  std::cout << "number_of_point_records : " << number_of_point_records << std::endl;
  std::cout << "x_scale_factor : " << x_scale_factor << std::endl;
  std::cout << "y_scale_factor : " << y_scale_factor << std::endl;
  std::cout << "z_scale_factor : " << z_scale_factor << std::endl;
}
void las::WritePoint(std::ostream &os, PointXYZRGB point) {

  char unused[8] = {0};
  os.write((char*)&point.X, 4);
  os.write((char*)&point.Y, 4);
  os.write((char*)&point.Z, 4);
  os.write(unused, 8);
  uint16_t r, g, b;
  r = point.R * 255;
  g = point.G * 255;
  b = point.B * 255;
  os.write((char*)&r, 2);
  os.write((char*)&g, 2);
  os.write((char*)&b, 2);

}
