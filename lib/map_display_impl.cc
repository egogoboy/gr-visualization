/* -*- c++ -*- */
/*
 * Copyright 2025 egogoboy.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#include "map_display_impl.h"
#include <gnuradio/io_signature.h>
#include <qboxlayout.h>
#include <qnamespace.h>
#include <qtimer.h>
#include <qwidget.h>
#include <QApplication>
#include <thread>

namespace gr {
namespace visualization {

using input_type = float;
using output_type = float;

map_display::sptr map_display::make()
{
    return gnuradio::make_block_sptr<map_display_impl>();
}


/*
 * The private constructor
 */
map_display_impl::map_display_impl()
    : gr::block("map_display",
                gr::io_signature::make(0, 0, sizeof(input_type)),
                gr::io_signature::make(0, 0, sizeof(output_type)))
{
    int argc = 0;
    char** argv = nullptr;
    static QApplication* app = nullptr;
    if (!QApplication::instance()) {
        app = new QApplication(argc, argv);
    }

    message_port_register_in(pmt::mp("in"));
    set_msg_handler(pmt::mp("in"), [this](pmt::pmt_t msg) { handle_msg(msg); });

    window_ = new QWidget();
    window_->setWindowTitle("Flight Display");

    window_->setLayout(createLayout());

    window_->resize(800, 600);
    window_->show();

    timer_ = new QTimer(window_);
    QObject::connect(timer_, &QTimer::timeout, [this]() {
        this->process_queue();
        QCoreApplication::processEvents();
    });
    timer_->start(100);
}

void map_display_impl::process_queue()
{
    std::vector<telemetry_msg> local;
    {
        std::lock_guard<std::mutex> lock(received_messages_mutex_);
        while (!received_messages_.empty()) {
            local.push_back(std::move(received_messages_.front()));
            received_messages_.pop();
        }
    }

    if (local.empty())
        return;

    // можно обрабатывать все сообщения или только последний (например, последний)
    // тут обновляем UI по последнему сообщению (чтобы не перегружать интерфейс)
    const telemetry_msg& last = local.back();
    update_coordinates(last.lat, last.lon);
    update_drift_label(last.yaw, last.cog); // TEST
    update_attitude_indicator(last.pitch, last.roll);
    update_speed_indicator(last.sog);
    update_heading_indicator(last.yaw);
}

QBoxLayout* map_display_impl::createLayout()
{
    QHBoxLayout* layout = new QHBoxLayout(window_);
    QVBoxLayout* layout_indicators = new QVBoxLayout();
    QVBoxLayout* layout_map = new QVBoxLayout();

    ai_ = new qfi_AI(window_);
    asi_ = new qfi_ASI(window_);
    hi_ = new qfi_HI(window_);
    mw_ = new map_widget(window_);

    ai_->setFixedSize(200, 200);
    asi_->setFixedSize(200, 200);
    hi_->setFixedSize(200, 200);

    ai_->setInteractive(false);

    label_ = new QLabel("Lat: --, Lon: --", window_);
    drift_label_ = new QLabel("Drift: --", window_);

    layout_indicators->addWidget(ai_);
    layout_indicators->addWidget(asi_);
    layout_indicators->addWidget(hi_);
    layout_indicators->addWidget(drift_label_);

    layout_map->addWidget(mw_, 1);
    layout_map->addWidget(label_);

    layout_indicators->addStretch();
    layout->addLayout(layout_map);
    layout->addLayout(layout_indicators);

    return layout;
}

void map_display_impl::update_coordinates(double lat, double lon)
{
    if (label_) {
        QString text = QString("Lat: %1\nLon: %2").arg(lat).arg(lon);
        label_->setText(text);
    }

    if (mw_) {
        mw_->updatePosition(lat, lon);
    }
}

void map_display_impl::update_drift_label(double heading, double cog)
{
    if (drift_label_) {
        QString text = QString("Drift: %1\n").arg(cog - heading);
        drift_label_->setText(text);
    }
}

void map_display_impl::update_attitude_indicator(double pitch, double roll)
{
    if (ai_) {
        ai_->setPitch(pitch);
        ai_->setRoll(roll);

        ai_->redraw();
    }
}

void map_display_impl::update_speed_indicator(double speed)
{
    if (asi_) {
        asi_->setAirspeed(speed);
        asi_->redraw();
    }
}

void map_display_impl::update_heading_indicator(double yaw)
{
    if (hi_) {
        double heading = fmod((yaw < 0 ? yaw + 360.0 : yaw), 360.0);
        hi_->setHeading(heading);

        hi_->redraw();
    }
}

/*
 * Our virtual destructor.
 */
map_display_impl::~map_display_impl()
{
    if (timer_) {
        timer_->stop();
        timer_->deleteLater();
        timer_ = nullptr;
    }

    if (window_) {
        QMetaObject::invokeMethod(window_, "close", Qt::QueuedConnection);
        window_->deleteLater();
        window_ = nullptr;
    }
}

void map_display_impl::handle_msg(pmt::pmt_t msg)
{
    if (!pmt::is_dict(msg)) {
        print_log("received non-dictionary message");
        return;
    }

    telemetry_msg tmsg;

    auto get_double = [&](const char* key, double def = 0.0) -> double {
        pmt::pmt_t val = pmt::dict_ref(msg, pmt::intern(key), pmt::PMT_NIL);
        if (pmt::is_real(val))
            return pmt::to_double(val);
        if (pmt::is_integer(val))
            return static_cast<double>(pmt::to_long(val));
        return def;
    };

    auto get_int = [&](const char* key, int def = 0) -> int {
        pmt::pmt_t val = pmt::dict_ref(msg, pmt::intern(key), pmt::PMT_NIL);
        if (pmt::is_integer(val))
            return pmt::to_long(val);
        if (pmt::is_real(val))
            return static_cast<int>(pmt::to_double(val));
        return def;
    };

    tmsg.timestamp = get_double("timestamp");
    tmsg.lat = get_double("lat");
    tmsg.lon = get_double("lon");
    tmsg.alt = get_double("alt");
    tmsg.sog = get_double("sog");
    tmsg.cog = get_double("cog");
    tmsg.vx = get_double("vx");
    tmsg.vy = get_double("vy");
    tmsg.pitch = get_double("pitch");
    tmsg.roll = get_double("roll");
    tmsg.yaw = get_double("yaw");
    tmsg.valid = get_int("valid", 0);

    {
        std::lock_guard<std::mutex> lock(received_messages_mutex_);
        received_messages_.push(std::move(tmsg));
    }
}


void map_display_impl::print_log(const std::string& msg)
{
    std::cerr << LOG_LABEL << " " << msg << std::endl;
}

void map_display_impl::forecast(int noutput_items, gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
}

int map_display_impl::general_work(int noutput_items,
                                   gr_vector_int& ninput_items,
                                   gr_vector_const_void_star& input_items,
                                   gr_vector_void_star& output_items)
{
    // auto in = static_cast<const input_type*>(input_items[0]);
    // auto out = static_cast<output_type*>(output_items[0]);

    // Do <+signal processing+>
    // Tell runtime system how many input items we consumed on
    // each input stream.
    consume_each(noutput_items);

    // Tell runtime system how many output items we produced.
    return noutput_items;
}

} // namespace visualization
} /* namespace gr */
