/* -*- c++ -*- */
/*
 * Copyright 2025 egogoboy.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#ifndef INCLUDED_VISUALIZATION_MAP_DISPLAY_H
#define INCLUDED_VISUALIZATION_MAP_DISPLAY_H

#include <gnuradio/visualization/api.h>
#include <gnuradio/block.h>

namespace gr {
  namespace visualization {

    /*!
     * \brief <+description of block+>
     * \ingroup visualization
     *
     */
    class VISUALIZATION_API map_display : virtual public gr::block
    {
     public:
      typedef std::shared_ptr<map_display> sptr;

      /*!
       * \brief Return a shared_ptr to a new instance of visualization::map_display.
       *
       * To avoid accidental use of raw pointers, visualization::map_display's
       * constructor is in a private implementation
       * class. visualization::map_display::make is the public interface for
       * creating new instances.
       */
      static sptr make();
    };

  } // namespace visualization
} // namespace gr

#endif /* INCLUDED_VISUALIZATION_MAP_DISPLAY_H */
