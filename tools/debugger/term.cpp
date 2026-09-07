#include "term.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <sstream>

#include <unistd.h>

namespace rf::dbg {

bool Style::enabled = false;

const char* Style::reset()  { return enabled ? "\033[0m" : ""; }
const char* Style::bold()   { return enabled ? "\033[1m" : ""; }
const char* Style::dim()    { return enabled ? "\033[2m" : ""; }
const char* Style::red()    { return enabled ? "\033[31m" : ""; }
const char* Style::yellow() { return enabled ? "\033[33m" : ""; }
const char* Style::green()  { return enabled ? "\033[32m" : ""; }
const char* Style::cyan()   { return enabled ? "\033[36m" : ""; }
const char* Style::grey()   { return enabled ? "\033[90m" : ""; }

const char* Style::of(Severity s) {
    switch (s) {
        case Severity::Ok: return green();
        case Severity::Notice: return cyan();
        case Severity::Warning: return yellow();
        case Severity::Fault: return red();
    }
    return "";
}

bool stdout_is_tty() { return ::isatty(STDOUT_FILENO) == 1; }

void init_style(bool force_no_color) {
    if (force_no_color) { Style::enabled = false; return; }
    // https://no-color.org/ — any non-empty value disables colour.
    const char* nc = std::getenv("NO_COLOR");
    if (nc != nullptr && nc[0] != '\0') { Style::enabled = false; return; }
    const char* term = std::getenv("TERM");
    if (term != nullptr && std::string(term) == "dumb") { Style::enabled = false; return; }
    Style::enabled = stdout_is_tty();
}

std::string num(double v, int decimals, int width) {
    std::string out;
    if (!std::isfinite(v)) {
        // Missing is missing. Rendering it as 0 is how a dead wheel reads as
        // a stationary one.
        out = "--";
    } else {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "%.*f", decimals, v);
        out = buf;
    }
    return width > 0 ? lpad(out, width) : out;
}

std::size_t display_width(const std::string& s) {
    std::size_t w = 0;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i] == '\033') {
            // Skip an SGR sequence: ESC [ ... m
            while (i < s.size() && s[i] != 'm') ++i;
            continue;
        }
        // Count UTF-8 lead bytes only, so multi-byte glyphs count as one.
        if ((static_cast<unsigned char>(s[i]) & 0xC0) != 0x80) ++w;
    }
    return w;
}

std::string rpad(const std::string& s, int width) {
    const std::size_t w = display_width(s);
    if (static_cast<int>(w) >= width) return s;
    return s + std::string(width - w, ' ');
}

std::string lpad(const std::string& s, int width) {
    const std::size_t w = display_width(s);
    if (static_cast<int>(w) >= width) return s;
    return std::string(width - w, ' ') + s;
}

Table::Table(std::vector<std::string> headers) : headers_(std::move(headers)) {}

void Table::row(std::vector<std::string> cells) {
    rows_.push_back(Row{std::move(cells), false});
}

void Table::rule() { rows_.push_back(Row{{}, true}); }

int Table::line_count() const {
    // header + underline + rows
    return 2 + static_cast<int>(rows_.size());
}

void Table::print() const {
    const std::size_t ncol = headers_.size();
    std::vector<int> width(ncol, 0);
    for (std::size_t c = 0; c < ncol; ++c) {
        width[c] = static_cast<int>(display_width(headers_[c]));
    }
    for (const auto& r : rows_) {
        if (r.is_rule) continue;
        for (std::size_t c = 0; c < ncol && c < r.cells.size(); ++c) {
            width[c] = std::max(width[c], static_cast<int>(display_width(r.cells[c])));
        }
    }

    std::ostringstream out;
    out << Style::bold();
    for (std::size_t c = 0; c < ncol; ++c) {
        out << rpad(headers_[c], width[c]) << (c + 1 < ncol ? "  " : "");
    }
    out << Style::reset() << '\n';

    int total = 0;
    for (std::size_t c = 0; c < ncol; ++c) total += width[c] + (c + 1 < ncol ? 2 : 0);
    out << Style::grey() << std::string(total, '-') << Style::reset() << '\n';

    for (const auto& r : rows_) {
        if (r.is_rule) {
            out << Style::grey() << std::string(total, '-') << Style::reset() << '\n';
            continue;
        }
        for (std::size_t c = 0; c < ncol; ++c) {
            const std::string cell = c < r.cells.size() ? r.cells[c] : std::string();
            out << rpad(cell, width[c]) << (c + 1 < ncol ? "  " : "");
        }
        out << '\n';
    }
    std::cout << out.str();
}

void FrameWriter::begin_frame() {
    if (first_) { first_ = false; return; }
    if (stdout_is_tty() && last_lines_ > 0) {
        // Move up and clear, so the frame redraws in place.
        std::cout << "\033[" << last_lines_ << "A";
        for (int i = 0; i < last_lines_; ++i) std::cout << "\033[2K\033[1B";
        std::cout << "\033[" << last_lines_ << "A";
    } else {
        // Piped or redirected: let frames accumulate, separated by a blank
        // line, so the capture stays readable.
        std::cout << '\n';
    }
}

void FrameWriter::end_frame(int lines_written) {
    last_lines_ = lines_written;
    std::cout.flush();
}

namespace {
std::atomic<bool> g_interrupted{false};
extern "C" void on_signal(int) { g_interrupted.store(true); }
}  // namespace

void install_signal_handlers() {
    // Deliberately not SA_RESETHAND: a second Ctrl-C should also be caught,
    // because the stop path must run even if the operator is impatient.
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
}

bool interrupted() { return g_interrupted.load(); }

}  // namespace rf::dbg
