#ifndef CSV_READER_H
#define CSV_READER_H

#include <cmath>
#include <string>
#include <tuple>
#include <vector>

namespace csv_reader {

/* Count the number of lines (samples) in a CSV file. */
std::tuple<bool, size_t> count_lines(const std::string& file_path);

/* Parse a numeric CSV file into a row-major buffer.
   num_columns_hint is used for pre-allocation; it does not enforce column count. */
std::tuple<bool, std::string> parse_numeric_csv(
    const std::string& file_path,
    size_t num_columns_hint,
    std::vector<std::vector<double_t>>& buffer);

} // namespace csv_reader

#endif // CSV_READER_H
