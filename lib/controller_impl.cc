/* -*- c++ -*- */
/*
 * Copyright 2017 Pieter Robyns.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "controller_impl.h"
#include "channelizer_impl.h"

namespace gr {
  namespace lora {

    controller::sptr
    controller::make(void* parent) {
      return gnuradio::get_initial_sptr
        (new controller_impl(parent));
    }

    /*
     * The private constructor
     */
    controller_impl::controller_impl(void* parent)
      : gr::block("controller",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(0, 0, 0)) {
        d_parent = parent;
        d_port = pmt::intern("control");
        message_port_register_in(d_port);
        set_msg_handler(d_port, boost::bind(&controller_impl::handle_control, this, _1));

	
        d_portha = pmt::intern("hahain");//lx
        message_port_register_in(d_portha);//lx
        set_msg_handler(d_portha, boost::bind(&controller_impl::handle_controlha, this, _1));//lx
        
        
        message_port_register_out(pmt::mp("haha_backout"));

    }

    void controller_impl::handle_control(pmt::pmt_t msg){
        if(pmt::symbol_to_string(pmt::car(msg)).compare("cfo") == 0) {
            std::cout << "Setting CFO " << pmt::to_double(pmt::cdr(msg)) << std::endl; 
            ((channelizer_impl*)d_parent)->apply_cfo(pmt::to_double(pmt::cdr(msg)));   // TODO: Pretty hacky cast, can we do this in a cleaner way?
        }
    }

    void controller_impl::handle_controlha(pmt::pmt_t msg){//lx
        std::cout << "hehe,controler coming"<< std::endl;
        float BW_ok=0;
        if(pmt::symbol_to_string(pmt::car(msg)).compare("BW") == 0) {
            std::cout << "Setting BW " << pmt::to_double(pmt::cdr(msg)) << std::endl; 
            if(pmt::to_double(pmt::cdr(msg))!=0){
              std::cout << "1"<< std::endl;
              ((channelizer_impl*)d_parent)->apply_BW(pmt::to_double(pmt::cdr(msg)));
              BW_ok=1.0;
            }
            else{
              std::cout << "2"<< std::endl;
              BW_ok=0.0;
            }
            pmt::pmt_t payload_bloblx = pmt::cons(pmt::intern(std::string("BW_ok")), pmt::from_double(BW_ok));
            message_port_pub(pmt::mp("haha_backout"), payload_bloblx);//lx
        }
    }

    /*
     * Our virtual destructor.
     */
    controller_impl::~controller_impl()
    {
    }


  } /* namespace lora */
} /* namespace gr */
