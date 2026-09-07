// Terminal rendering for the debugger — SSH-safe by construction.
//
// No ncurses, no alternate screen, no cursor addressing beyond "move up N
// lines". That keeps it usable over a plain ssh session, inside tmux, through
// a pipe, and in a CI log. Everything degrades in one direction: if stdout is
// not a tty, colour is off and the live view appends whole frames instead of
// redrawing in place, so `debugger monitor | tee log.txt` produces a readable
// file rather than a screenful of escape codes.
//
// The number formatting here is a safety feature, not cosmetics. Every moteus
// telemetry field is NaN-initialised and a reply that arrives may still carry
// missing fields, so a "0.00" that actually means "no data" is exactly the
// class of bug the audit found in the wheel-odometry path. num() renders
// non-finite as "--" and never as a number.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "decode.h"

namespace rf::dbg {

// ANSI SGR codes, empty strings when colour is disabled.
struct Style {
    static bool enabled;

    static const char* reset();
    static const char* bold();
    static const char* dim();
    static const char* red();
    static const char* yellow();
    static const char* green();
    static const char* cyan();
    static const char* grey();

    // Colour for a severity, so every view agrees on what "bad" looks like.
    static const char* of(Severity s);
};

// True when stdout is a terminal. Drives colour and in-place redraw defaults.
bool stdout_is_tty();

// Enable colour unless forced off or stdout is not a tty. Honours NO_COLOR.
void init_style(bool force_no_color);

// Format a double to `decimals` places, or "--" when non-finite.
// Never emits "0" for missing data.
std::string num(double v, int decimals = 3, int width = 0);

// Right-align `s` into `width`. Does not truncate; a long value pushes the
// column rather than silently losing digits.
std::string rpad(const std::string& s, int width);
std::string lpad(const std::string& s, int width);

// A simple fixed-column table. Columns size to their widest cell.
class Table {
public:
    explicit Table(std::vector<std::string> headers);

    // Cells may carry ANSI codes; display width ignores them.
    void row(std::vector<std::string> cells);
    void rule();  // horizontal separator at this point

    // Rendered line count, so the live view knows how far to move the cursor.
    int line_count() const;
    void print() const;

private:
    struct Row {
        std::vector<std::string> cells;
        bool is_rule = false;
    };
    std::vector<std::string> headers_;
    std::vector<Row> rows_;
};

// Visible width of a string, ignoring ANSI SGR sequences.
std::size_t display_width(const std::string& s);

// Live-view frame control. When stdout is a tty, begin_frame() moves the
// cursor up over the previous frame so the screen updates in place; when it
// is not, it emits a blank line and lets the frame scroll.
class FrameWriter {
public:
    void begin_frame();
    void end_frame(int lines_written);

private:
    int last_lines_ = 0;
    bool first_ = true;
};

// Install SIGINT/SIGTERM handlers that set a flag rather than killing the
// process outright, so callers can stop the wheels before exiting.
void install_signal_handlers();
bool interrupted();

}  // namespace rf::dbg
