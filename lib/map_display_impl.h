/* -*- c++ -*- */
/*
 * Copyright 2025 egogoboy.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_VISUAL_MAP_DISPLAY_IMPL_H
#define INCLUDED_VISUAL_MAP_DISPLAY_IMPL_H

#include "map_widget.h"
#include <gnuradio/visualization/map_display.h>
#include <gnuradio/visualization/telemetry_msg.h>
#include <qboxlayout.h>
#include <qfi_AI.h>
#include <qfi_ASI.h>
#include <qfi_HI.h>
#include <qlabel.h>
#include <qtimer.h>
#include <qwidget.h>
#include <mutex>
#include <queue>

namespace gr {
namespace visualization {


class map_display_impl : public map_display
{

public:
    map_display_impl();
    ~map_display_impl();

    // Where all the action really happens
    void forecast(int noutput_items, gr_vector_int& ninput_items_required);

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

private:
    void process_queue();
    void handle_msg(pmt::pmt_t msg);

    void update_coordinates(double lat, double lon);
    void update_drift_label(double heading, double cog);
    void update_attitude_indicator(double pitch, double roll);
    void update_speed_indicator(double speed);
    void update_heading_indicator(double yaw);

    QBoxLayout* createLayout();
    void print_log(const std::string& msg);

    std::queue<telemetry_msg> received_messages_;
    std::mutex received_messages_mutex_;

    QWidget* window_;
    QTimer* timer_;
    QLabel* label_;
    QLabel* drift_label_;

    qfi_AI* ai_;
    qfi_ASI* asi_;
    qfi_HI* hi_;
    map_widget* mw_;

    const std::string LOG_LABEL = "[map display]";
};

} // namespace visualization
} // namespace gr

#endif /* INCLUDED_VISUAL_MAP_DISPLAY_IMPL_H */
