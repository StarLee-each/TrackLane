//
// Created by Kurosu Chan on 2023/10/16.
//

#include "simple_log.h"
#include <cstdio>
#include <chrono>

namespace simple_log {
using tp            = std::chrono::steady_clock::time_point;
using disp_duration = std::chrono::duration<tp::duration::rep, std::milli>;
tp start            = tp::min();

tp get_start() {
  if (start == tp::min()) {
    start = std::chrono::steady_clock::now();
    return start;
  } else {
    return start;
  }
}

void init() {
  get_start();
}

void print_prefix(char symbol, const char *tag, const char *file_name, int line_number) {
  printf("[%c]", symbol);

  if (tag != nullptr) {
    printf("[%s]", tag);
  }

  auto now  = std::chrono::steady_clock::now();
  auto time = now - get_start();
  // tp::duration::rep
  printf("[%lld]", std::chrono::duration_cast<disp_duration>(time).count());

  if (file_name != nullptr) {
    printf("[%s:%d]", file_name, line_number);
  }
  putchar('\t');
}
}

