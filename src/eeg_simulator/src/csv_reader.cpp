#include "csv_reader.h"

#include <algorithm>
#include <cstdint>
#include <cstring>

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

namespace csv_reader {

namespace {

/* RAII wrapper for a memory-mapped file. */
class MappedFile {
public:
  MappedFile(const std::string& path) {
    fd_ = open(path.c_str(), O_RDONLY);
    if (fd_ == -1) return;

    struct stat st;
    if (fstat(fd_, &st) == -1) {
      close(fd_);
      fd_ = -1;
      return;
    }
    size_ = static_cast<size_t>(st.st_size);

    if (size_ == 0) {
      close(fd_);
      fd_ = -1;
      data_ = nullptr;
      return;
    }

    data_ = static_cast<const char*>(
        mmap(nullptr, size_, PROT_READ, MAP_PRIVATE | MAP_NORESERVE, fd_, 0));
    close(fd_);
    fd_ = -1;

    if (data_ == MAP_FAILED) {
      data_ = nullptr;
      size_ = 0;
      return;
    }

    madvise(const_cast<char*>(data_), size_, MADV_SEQUENTIAL | MADV_WILLNEED);
  }

  ~MappedFile() {
    if (data_) {
      munmap(const_cast<char*>(data_), size_);
    }
  }

  MappedFile(const MappedFile&) = delete;
  MappedFile& operator=(const MappedFile&) = delete;

  bool is_open() const { return data_ != nullptr; }
  const char* data() const { return data_; }
  size_t size() const { return size_; }

private:
  int fd_ = -1;
  const char* data_ = nullptr;
  size_t size_ = 0;
};

} // anonymous namespace

std::tuple<bool, size_t> count_lines(const std::string& file_path) {
  MappedFile file(file_path);

  if (file.size() == 0 && !file.is_open()) {
    /* Distinguish empty file (OK) from open failure.
       If size is 0 and data is null, the file either doesn't exist or is empty.
       Try a simple open to differentiate. */
    int fd = open(file_path.c_str(), O_RDONLY);
    if (fd == -1) {
      return std::make_tuple(false, 0);
    }
    close(fd);
    return std::make_tuple(true, 0);
  }

  if (!file.is_open()) {
    return std::make_tuple(false, 0);
  }

  const char* data = file.data();
  size_t size = file.size();
  size_t line_count = 0;

  for (size_t i = 0; i < size; i++) {
    if (data[i] == '\n') {
      line_count++;
    }
  }

  /* Count last line if file doesn't end with newline. */
  if (data[size - 1] != '\n') {
    line_count++;
  }

  return std::make_tuple(true, line_count);
}

std::tuple<bool, std::string> parse_numeric_csv(
    const std::string& file_path,
    size_t num_columns_hint,
    std::vector<std::vector<double_t>>& buffer) {

  MappedFile file(file_path);

  if (file.size() == 0 && !file.is_open()) {
    int fd = open(file_path.c_str(), O_RDONLY);
    if (fd == -1) {
      return std::make_tuple(false, "Error opening file: " + file_path);
    }
    close(fd);
    buffer.clear();
    return std::make_tuple(true, "");
  }

  if (!file.is_open()) {
    return std::make_tuple(false, "Error memory-mapping file: " + file_path);
  }

  const char* data = file.data();
  const char* end = data + file.size();

  /* Estimate row count for pre-allocation. */
  size_t cols = std::max(num_columns_hint, static_cast<size_t>(1));
  size_t estimated_bytes_per_row = cols * 8 + cols;
  size_t estimated_rows = file.size() / estimated_bytes_per_row;

  buffer.clear();
  buffer.reserve(estimated_rows + estimated_rows / 10);

  const char* pos = data;
  uint32_t line_number = 0;

  while (pos < end) {
    line_number++;

    const char* line_end = static_cast<const char*>(memchr(pos, '\n', end - pos));
    if (!line_end) {
      line_end = end;
    }

    /* Skip empty lines. */
    if (pos == line_end || (pos + 1 == line_end && *pos == '\r')) {
      pos = line_end + 1;
      continue;
    }

    std::vector<double_t> row;
    row.reserve(cols);

    const char* field_start = pos;
    while (field_start < line_end) {
      char* parse_end;
      double_t value = strtod(field_start, &parse_end);

      if (parse_end == field_start) {
        return std::make_tuple(false,
            "Error converting string to double on line " + std::to_string(line_number));
      }

      row.push_back(value);

      field_start = parse_end;
      if (field_start < line_end && (*field_start == ',' || *field_start == '\r')) {
        field_start++;
      }
    }

    buffer.push_back(std::move(row));
    pos = line_end + 1;
  }

  return std::make_tuple(true, "");
}

} // namespace csv_reader
