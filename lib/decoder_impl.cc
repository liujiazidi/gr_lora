/* -*- c++ -*- */
/*
 * Copyright 2017 Pieter Robyns, William Thenaers.
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
#include <gnuradio/expj.h>
#include <liquid/liquid.h>
#include <numeric>
#include <algorithm>
#include <lora/loratap.h>
#include <lora/utilities.h>
#include "decoder_impl.h"
#include "tables.h"
#include <signal.h>
#include <setjmp.h>
#include <stdarg.h>
#include <time.h>
jmp_buf env;

#include "dbugr.hpp"
#define DEBUG 1
#define numbles_for_SF    8192
#define numbers_para 10
struct timeval dwStart;  
struct timeval dwEnd; 
unsigned long dwTime=0;

namespace gr {
    namespace lora {

        decoder::sptr decoder::make(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, bool disable_drift_correction) {
            return gnuradio::get_initial_sptr
                   (new decoder_impl(samp_rate, bandwidth, sf, implicit, cr, crc, reduced_rate, disable_drift_correction));
        }
        
         void recvSignal(int sig)  
        {  
            printf("received signal %d !!!\n",sig);
             siglongjmp(env,1);
        }  

        /**
         * The private constructor
         */
        decoder_impl::decoder_impl(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, bool disable_drift_correction)
            : gr::sync_block("decoder",
                             gr::io_signature::make(1, -1, sizeof(gr_complex)),
                             gr::io_signature::make(0, 0, 0)),
            d_pwr_queue(MAX_PWR_QUEUE_SIZE) {
            // Radio config
            d_state = gr::lora::DecoderState::DETECT;

            if (sf < 6 || sf > 13) {
                std::cerr << "[LoRa Decoder] ERROR : Spreading factor should be between 6 and 12 (inclusive)!" << std::endl
                          << "                       Other values are currently not supported." << std::endl;
                exit(1);
            }

            #ifdef DEBUG
                d_debug_samples.open("/tmp/grlora_debug", std::ios::out | std::ios::binary);
                d_debug.open("/tmp/grlora_debug_txt", std::ios::out);
                d_dbg.attach();
            #endif

                
            set_init(samp_rate, bandwidth, sf, implicit, cr, crc, reduced_rate, disable_drift_correction);
                
            //std::cout<<"try1"<<std::endl;
                
            upchirp71_len=ideal_chirps_num(2000000,125000,7);
            d_upchirp71.resize(upchirp71_len);
            build_ideal_chirps_sf_bw(2000000,125000,7,&d_upchirp71[0]);
            auto_corr71=xcorr(&d_upchirp71[0], &d_upchirp71[0], NULL,  upchirp71_len,upchirp71_len);
                
            upchirp72_len=ideal_chirps_num(2000000,250000,7);
            d_upchirp72.resize(upchirp72_len);
            build_ideal_chirps_sf_bw(2000000,250000,7,&d_upchirp72[0]);
            auto_corr72=xcorr(&d_upchirp72[0], &d_upchirp72[0], NULL,  upchirp72_len,upchirp72_len);
                
            upchirp81_len=ideal_chirps_num(2000000,125000,8);
            d_upchirp81.resize(upchirp81_len);
            build_ideal_chirps_sf_bw(2000000,125000,8,&d_upchirp81[0]);
            auto_corr81=xcorr(&d_upchirp81[0], &d_upchirp81[0], NULL,  upchirp81_len,upchirp81_len);
                
            upchirp82_len=ideal_chirps_num(2000000,250000,8);
            d_upchirp82.resize(upchirp82_len);
            build_ideal_chirps_sf_bw(2000000,250000,8,&d_upchirp82[0]);
            auto_corr82=xcorr(&d_upchirp82[0], &d_upchirp82[0], NULL,  upchirp82_len,upchirp82_len);
                
            upchirp85_len=ideal_chirps_num(2000000,500000,8);
            d_upchirp85.resize(upchirp85_len);
            build_ideal_chirps_sf_bw(2000000,500000,8,&d_upchirp85[0]);
            auto_corr85=xcorr(&d_upchirp85[0], &d_upchirp85[0], NULL,  upchirp85_len,upchirp85_len);
                
            upchirp91_len=ideal_chirps_num(2000000,125000,9);
            d_upchirp91.resize(upchirp91_len);
            build_ideal_chirps_sf_bw(2000000,125000,9,&d_upchirp91[0]);
            auto_corr91=xcorr(&d_upchirp91[0], &d_upchirp91[0], NULL,  upchirp91_len,upchirp91_len);
                
            upchirp92_len=ideal_chirps_num(2000000,250000,9);
            d_upchirp92.resize(upchirp92_len);
            build_ideal_chirps_sf_bw(2000000,250000,9,&d_upchirp92[0]);
            auto_corr92=xcorr(&d_upchirp92[0], &d_upchirp92[0], NULL,  upchirp92_len,upchirp92_len);
                
            upchirp95_len=ideal_chirps_num(2000000,500000,9);
            d_upchirp95.resize(upchirp95_len);
            build_ideal_chirps_sf_bw(2000000,500000,9,&d_upchirp95[0]);
            auto_corr95=xcorr(&d_upchirp95[0], &d_upchirp95[0], NULL,  upchirp95_len,upchirp95_len);
                
            upchirp102_len=ideal_chirps_num(2000000,250000,10);
            d_upchirp102.resize(upchirp102_len);
            build_ideal_chirps_sf_bw(2000000,250000,10,&d_upchirp102[0]);
            auto_corr102=xcorr(&d_upchirp102[0], &d_upchirp102[0], NULL,  upchirp102_len,upchirp102_len);
                
            upchirp105_len=ideal_chirps_num(2000000,500000,10);
            d_upchirp105.resize(upchirp105_len);
            build_ideal_chirps_sf_bw(2000000,500000,10,&d_upchirp105[0]);
            auto_corr105=xcorr(&d_upchirp105[0], &d_upchirp105[0], NULL,  upchirp105_len,upchirp105_len);
                
            //std::cout<<"try2 "<<std::endl;
                
            std::cout << "Bits (nominal) per symbol: \t"      << d_bits_per_symbol    << std::endl;
            std::cout << "Bins per symbol: \t"      << d_number_of_bins     << std::endl;
            std::cout << "Samples per symbol: \t"   << d_samples_per_symbol << std::endl;
            std::cout << "Decimation: \t\t"         << d_decim_factor       << std::endl;
            comeback_flag=1;
            resetconfig=0;
           
            // Register gnuradio ports
            message_port_register_out(pmt::mp("frames"));
            message_port_register_out(pmt::mp("control"));
            message_port_register_out(pmt::mp("hahaout"));//lx
            
            d_portha_de = pmt::intern("haha_backin");//lx
            message_port_register_in(d_portha_de);//lx
            set_msg_handler(d_portha_de, boost::bind(&decoder_impl::handle_controlha_back, this, _1));//lx
    }

        void decoder_impl::handle_controlha_back(pmt::pmt_t msg){//lx
            std::cout << "hehe,welcome back"<< std::endl; 
            if(pmt::symbol_to_string(pmt::car(msg)).compare("BW_ok") == 0) {
                if(pmt::to_double(pmt::cdr(msg))==1){
                    //std::cout << "Setting back " << pmt::to_double(pmt::cdr(msg)) << std::endl; 
                    comeback_flag=1;
                }
                else{
                    resetconfig=1;
                }
             }
         }
        
        /**
         * Our virtual destructor.
         */
        decoder_impl::~decoder_impl() {
            #ifdef DEBUG
                if (d_debug_samples.is_open())
                    d_debug_samples.close();

                if (d_debug.is_open())
                    d_debug.close();
            #endif

            fft_destroy_plan(d_q);
            fft_destroy_plan(d_qr);
            fec_destroy(d_h48_fec);
        }
        

    void decoder_impl::set_init(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, bool disable_drift_correction){
        d_bw                 = bandwidth;
            d_implicit           = implicit;
            d_reduced_rate       = reduced_rate;
            d_phdr.cr            = cr;
            d_phdr.has_mac_crc   = crc;
            d_samples_per_second = samp_rate;
            d_payload_symbols    = 0;
            d_cfo_estimation     = 0.0f;
            d_dt                 = 1.0f / d_samples_per_second;
            d_sf                 = sf;
            d_bits_per_second    = (double)d_sf * (double)(4.0 / (4.0 + d_phdr.cr)) / (1u << d_sf) * d_bw;
            d_symbols_per_second = (double)d_bw / (1u << d_sf);
            d_period             = 1.0f / (double)d_symbols_per_second;
            d_bits_per_symbol    = (double)(d_bits_per_second    / d_symbols_per_second);
            d_samples_per_symbol = (uint32_t)(d_samples_per_second / d_symbols_per_second);
            d_delay_after_sync   = d_samples_per_symbol / 4u;
            d_number_of_bins     = (uint32_t)(1u << d_sf);
            d_number_of_bins_hdr = (uint32_t)(1u << (d_sf-2));
            d_decim_factor       = d_samples_per_symbol / d_number_of_bins;
            d_energy_threshold   = 0.0f;
            d_whitening_sequence = gr::lora::prng_payload;
            d_fine_sync = 0;
            d_enable_fine_sync = !disable_drift_correction;//!false
            set_output_multiple(numbles_for_SF);//2*d_samples_per_symbol);
            
            signal_flag=0;//lx

            //std::cout << "Bits (nominal) per symbol: \t"      << d_bits_per_symbol    << std::endl;
            //std::cout << "Bins per symbol: \t"      << d_number_of_bins     << std::endl;
            //std::cout << "Samples per symbol: \t"   << d_samples_per_symbol << std::endl;
            //std::cout << "Decimation: \t\t"         << d_decim_factor       << std::endl;
            if(!d_enable_fine_sync) {
                std::cout << "Warning: clock drift correction disabled" << std::endl;
            }
            if(d_implicit) {
                std::cout << "CR: \t\t"         << (int)d_phdr.cr       << std::endl;
                std::cout << "CRC: \t\t"         << (int)d_phdr.has_mac_crc       << std::endl;
            }

            // Locally generated chirps
            build_ideal_chirps();

            // FFT decoding preparations
            d_fft.resize(d_samples_per_symbol);
            d_mult_hf.resize(d_samples_per_symbol);
            d_tmp.resize(d_number_of_bins);
            d_q  = fft_create_plan(d_samples_per_symbol, &d_mult_hf[0], &d_fft[0],     LIQUID_FFT_FORWARD, 0);
            d_qr = fft_create_plan(d_number_of_bins,     &d_tmp[0],     &d_mult_hf[0], LIQUID_FFT_BACKWARD, 0);

            // Hamming coding
            fec_scheme fs = LIQUID_FEC_HAMMING84;
            d_h48_fec = fec_create(fs, NULL);

    }
        
        float decoder_impl::xcorr(const gr_complex * signala, const gr_complex * signalb, gr_complex * result=NULL,  uint32_t Na=2,uint32_t Nb=2)
        {
            if(result==NULL){
                result = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            }
            gr_complex * signala_ext = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * signalb_ext = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * out_shifted = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * outa = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * outb = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * out = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * outb_conj = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            
            fftplan pa = fft_create_plan(Na+Nb-1, &signala_ext[0], &outa[0], LIQUID_FFT_FORWARD, 0);
            fftplan pb = fft_create_plan(Na+Nb-1, &signalb_ext[0], &outb[0], LIQUID_FFT_FORWARD, 0);
            fftplan px = fft_create_plan(Na+Nb-1, &out[0], &result[0], LIQUID_FFT_BACKWARD, 0);

            //zeropadding
            memset (signala_ext, 0, sizeof(gr_complex) * (Nb - 1));
            memcpy (signala_ext + (Nb - 1), signala, sizeof(gr_complex) * Na);
            memcpy (signalb_ext, signalb, sizeof(gr_complex) * Nb);
            memset (signalb_ext + Nb, 0, sizeof(gr_complex) * (Na - 1));

            fft_execute(pa);
            fft_execute(pb);
  
            volk_32fc_conjugate_32fc(outb_conj, outb, Na+Nb-1);
            gr_complex scale = 1.0/(Na+Nb-1);
            for (uint32_t i = 0; i < Na+Nb-1; i++)
                out[i] = outa[i] * outb_conj[i] * scale;

            fft_execute(px);

            fft_destroy_plan(pa);
            fft_destroy_plan(pb);
            fft_destroy_plan(px);

            free(signala_ext);
            free(signalb_ext);
            free(out_shifted);
            free(out);
            free(outa);
            free(outb);
            free(outb_conj);

            //fftw_cleanup();
            float * resule_float = (float *) malloc(sizeof(float) * (Na+Nb-1));
            memset (resule_float, 0, sizeof(float) * (Na+Nb-1));
            volk_32fc_magnitude_squared_32f(resule_float,result,Na+Nb-1);
            float max_corr=*std::max_element(resule_float,resule_float+Na+Nb-1);
            
            return std::sqrt(max_corr);
        }

        void decoder_impl::build_ideal_chirps(void) {
            d_downchirp.resize(d_samples_per_symbol);
            d_upchirp.resize(d_samples_per_symbol);
            d_downchirp_ifreq.resize(d_samples_per_symbol);
            d_upchirp_ifreq.resize(d_samples_per_symbol);
            d_upchirp_ifreq_v.resize(d_samples_per_symbol*3);
            gr_complex tmp[d_samples_per_symbol*3];

            const double T       = -0.5 * d_bw * d_symbols_per_second;
            const double f0      = (d_bw / 2.0);
            const double pre_dir = 2.0 * M_PI;
            double t;
            gr_complex cmx       = gr_complex(1.0f, 1.0f);

            for (uint32_t i = 0u; i < d_samples_per_symbol; i++) {
                // Width in number of samples = samples_per_symbol
                // See https://en.wikipedia.org/wiki/Chirp#Linear
                t = d_dt * i;
                d_downchirp[i] = cmx * gr_expj(pre_dir * t * (f0 + T * t));
                d_upchirp[i]   = cmx * gr_expj(pre_dir * t * (f0 + T * t) * -1.0f);
            }

            // Store instantaneous frequency
            instantaneous_frequency(&d_downchirp[0], &d_downchirp_ifreq[0], d_samples_per_symbol);
            instantaneous_frequency(&d_upchirp[0],   &d_upchirp_ifreq[0],   d_samples_per_symbol);

            samples_to_file("/home/lx/decode_lora//downchirp.bin", &d_downchirp[0], d_downchirp.size(), sizeof(gr_complex));
            samples_to_file("/home/lx/decode_lora//upchirp.bin",   &d_upchirp[0],   d_upchirp.size(),   sizeof(gr_complex));

            // Upchirp sequence
            memcpy(tmp, &d_upchirp[0], sizeof(gr_complex) * d_samples_per_symbol);
            memcpy(tmp+d_samples_per_symbol, &d_upchirp[0], sizeof(gr_complex) * d_samples_per_symbol);
            memcpy(tmp+d_samples_per_symbol*2, &d_upchirp[0], sizeof(gr_complex) * d_samples_per_symbol);
            instantaneous_frequency(tmp, &d_upchirp_ifreq_v[0], d_samples_per_symbol*3);
        }
        void decoder_impl::build_ideal_chirps_lx(float samp_rate,uint32_t bandwidth, uint8_t sf) {         //lx
            double dt_lx=1.0f / samp_rate;
            double symbols_per_second_lx=(double)bandwidth / (1u << sf);
            uint32_t samples_per_symbol_lx=(uint32_t)(samp_rate / symbols_per_second_lx);
            d_upchirplx.resize(samples_per_symbol_lx/4+1);
            d_upchirplx_len=samples_per_symbol_lx/4+1;

            const double T       = -0.5 * bandwidth * symbols_per_second_lx;
            const double f0      = (bandwidth / 2.0);
            const double pre_dir = 2.0 * M_PI;
            double t;
            gr_complex cmx       = gr_complex(1.0f, 1.0f);
            uint32_t j = 0u;
            for (uint32_t i = 0u; i < samples_per_symbol_lx;j++ ) {                
                t = dt_lx * i;
                d_upchirplx[j]   = cmx * gr_expj(pre_dir * t * (f0 + T * t) * -1.0f);
                i=i+4;
            }
                      
        }
        
        uint32_t decoder_impl::ideal_chirps_num(float samp_rate,uint32_t bandwidth, uint8_t sf) {         //lx
            double symbols_per_second_lx=(double)bandwidth / (1u << sf);
            uint32_t samples_per_symbol_lx=(uint32_t)(samp_rate / symbols_per_second_lx);
            return samples_per_symbol_lx;
        }
        
        void decoder_impl::build_ideal_chirps_sf_bw(float samp_rate,uint32_t bandwidth, uint8_t sf,gr_complex* d_upchirplx) {         //lx
            double dt_lx=1.0f / samp_rate;
            double symbols_per_second_lx=(double)bandwidth / (1u << sf);
            uint32_t samples_per_symbol_lx=(uint32_t)(samp_rate / symbols_per_second_lx);
            
            const double T       = -0.5 * bandwidth * symbols_per_second_lx;
            const double f0      = (bandwidth / 2.0);
            const double pre_dir = 2.0 * M_PI;
            double t;
            gr_complex cmx       = gr_complex(1.0f, 1.0f);
            for (uint32_t i = 0u; i < samples_per_symbol_lx;i++ ) {                
                t = dt_lx * i;
                d_upchirplx[i]   = cmx * gr_expj(pre_dir * t * (f0 + T * t) * -1.0f);
            }
            samples_to_file("/home/lx/decode_lora/d_upchirplx.bin", &d_upchirplx[0], samples_per_symbol_lx, sizeof(gr_complex));//}          
        }


        void decoder_impl::values_to_file(const std::string path, const unsigned char *v, const uint32_t length, const uint32_t ppm) {
            std::ofstream out_file;
            out_file.open(path.c_str(), std::ios::out | std::ios::app);

            for (uint32_t i = 0u; i < length; i++) {
                std::string tmp = gr::lora::to_bin(v[i], ppm);
                out_file.write(tmp.c_str(), tmp.length());
                out_file.write(" ", 1);
            }
            out_file.write("\n", 1);

            out_file.close();
        }

        void decoder_impl::samples_to_file(const std::string path, const gr_complex *v, const uint32_t length, const uint32_t elem_size) {
            #ifdef DEBUG
                std::ofstream out_file;
                out_file.open(path.c_str(), std::ios::out | std::ios::binary );

                //for(std::vector<gr_complex>::const_iterator it = v.begin(); it != v.end(); ++it) {
                for (uint32_t i = 0u; i < length; i++) {
                    out_file.write(reinterpret_cast<const char *>(&v[i]), elem_size);
                }

                out_file.close();
            #else
                (void) path;
                (void) v;
                (void) length;
                (void) elem_size;
            #endif
        }
        
        void decoder_impl::samples_to_file_add(const std::string path, const uint8_t *v,const uint8_t *config, const uint32_t length) {
            #ifdef DEBUG
                std::ofstream out_file;
                out_file.open(path.c_str(), std::ios::out | std::ios::app);
                char * pCh= new char[100];
                char * pCh1= new char[50];
                uint32_t ha_length=0;
                //std::cout<<"2"<<std::endl;
                sprintf(pCh1,"%s"," ");
                for (uint32_t i = 0u; i < 2; i++) {
                    //std::cout<<"21 "<<i<<std::endl;
                    sprintf(pCh1,"%s%02X ",pCh1,config[i]);
                    //std::cout<<"22"<<std::endl;
                }
                out_file.write(&pCh1[0],6);
                out_file.write("\n",1);
                //std::cout<<"4"<<std::endl;
                sprintf(pCh,"%s"," ");
                ha_length=length;
                if(ha_length>11){
                    ha_length=11;
                }
                for (uint32_t i = 0u; i < ha_length; i++) {
                    sprintf(pCh,"%s%02X ",pCh,v[i]);
                    std::cout<<pCh<<std::endl;
                }
                out_file.write(&pCh[1],3*ha_length);
                out_file.write("\n",1);
                out_file.write("\n",1);
                out_file.close();
                delete pCh;
                delete pCh1;
            #else
                (void) path;
                (void) v;
                (void) length;
                (void) elem_size;
            #endif
        }

        void decoder_impl::samples_debug(const gr_complex *v, const uint32_t length) {
            #ifdef DEBUG
                gr_complex start_indicator(0.0f, 32.0f);
                d_debug_samples.write(reinterpret_cast<const char *>(&start_indicator), sizeof(gr_complex));

                for (uint32_t i = 1u; i < length; i++) {
                    d_debug_samples.write(reinterpret_cast<const char *>(&v[i]), sizeof(gr_complex));
                }
            #else
                (void) v;
                (void) length;
            #endif
        }

        inline void decoder_impl::instantaneous_frequency(const gr_complex *in_samples, float *out_ifreq, const uint32_t window) {
            if (window < 2u) {
                std::cerr << "[LoRa Decoder] WARNING : window size < 2 !" << std::endl;
                return;
            }

            /* instantaneous_phase */
            for (uint32_t i = 1u; i < window; i++) {
                const float iphase_1 = std::arg(in_samples[i - 1]);
                      float iphase_2 = std::arg(in_samples[i]);

                // Unwrapped loops from liquid_unwrap_phase
                while ( (iphase_2 - iphase_1) >  M_PI ) iphase_2 -= 2.0f*M_PI;
                while ( (iphase_2 - iphase_1) < -M_PI ) iphase_2 += 2.0f*M_PI;

                out_ifreq[i - 1] = iphase_2 - iphase_1;
            }

            // Make sure there is no strong gradient if this value is accessed by mistake
            out_ifreq[window - 1] = out_ifreq[window - 2];
        }

        inline void decoder_impl::instantaneous_phase(const gr_complex *in_samples, float *out_iphase, const uint32_t window) {
            out_iphase[0] = std::arg(in_samples[0]);

            for (uint32_t i = 1u; i < window; i++) {
                out_iphase[i] = std::arg(in_samples[i]);
                // = the same as atan2(imag(in_samples[i]),real(in_samples[i]));

                // Unwrapped loops from liquid_unwrap_phase
                while ( (out_iphase[i] - out_iphase[i-1]) >  M_PI ) out_iphase[i] -= 2.0f*M_PI;
                while ( (out_iphase[i] - out_iphase[i-1]) < -M_PI ) out_iphase[i] += 2.0f*M_PI;
            }
        }

        float decoder_impl::cross_correlate_ifreq_fast(const float *samples_ifreq, const float *ideal_chirp, const uint32_t window) {
            float result = 0;
            volk_32f_x2_dot_prod_32f(&result, samples_ifreq, ideal_chirp, window);
            return result;
        }

        float decoder_impl::cross_correlate_fast(const gr_complex *samples, const gr_complex *ideal_chirp, const uint32_t window) {
            gr_complex result = 0;
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, samples, ideal_chirp, window);
            return abs(result);
        }

        float decoder_impl::cross_correlate(const gr_complex *samples_1, const gr_complex *samples_2, const uint32_t window) {
            float result = 0.0f;

            for (uint32_t i = 0u; i < window; i++) {
                result += std::real(samples_1[i] * std::conj(samples_2[i]));
            }

            result /= (float)window;

            return result;
        }

        float decoder_impl::cross_correlate_ifreq(const float *samples_ifreq, const std::vector<float>& ideal_chirp, const uint32_t to_idx) {
            float result = 0.0f;

            const float average   = std::accumulate(samples_ifreq  , samples_ifreq + to_idx, 0.0f) / (float)(to_idx);
            const float chirp_avg = std::accumulate(&ideal_chirp[0], &ideal_chirp[to_idx]  , 0.0f) / (float)(to_idx);
            const float sd        =   stddev(samples_ifreq   , to_idx, average)
                                    * stddev(&ideal_chirp[0] , to_idx, chirp_avg);

            for (uint32_t i = 0u; i < to_idx; i++) {
                result += (samples_ifreq[i] - average) * (ideal_chirp[i] - chirp_avg) / sd;
            }

            result /= (float)(to_idx);

            return result;
        }

        void decoder_impl::fine_sync(const gr_complex* in_samples, uint32_t bin_idx, int32_t search_space) {
            int32_t shift_ref = (bin_idx+1) * d_decim_factor;
            float samples_ifreq[d_samples_per_symbol];
            float max_correlation = 0.0f;
            int32_t lag = 0;

            instantaneous_frequency(in_samples, samples_ifreq, d_samples_per_symbol);

            for(int32_t i = -search_space+1; i < search_space; i++) {
                //float c = cross_correlate_fast(in_samples, &d_upchirp_v[shift_ref+i+d_samples_per_symbol], d_samples_per_symbol);
                float c = cross_correlate_ifreq_fast(samples_ifreq, &d_upchirp_ifreq_v[shift_ref+i+d_samples_per_symbol], d_samples_per_symbol);
                if(c > max_correlation) {
                     max_correlation = c;
                     lag = i;
                 }
            }

            #ifdef DEBUG
                //d_debug << "FINE: " << -lag << std::endl;
            #endif

            d_fine_sync = -lag;

            //if(abs(d_fine_sync) >= d_decim_factor / 2)
            //    d_fine_sync = 0;
            //d_fine_sync = 0;
        }

        float decoder_impl::detect_preamble_autocorr(const gr_complex *samples, const uint32_t window) {
            const gr_complex* chirp1 = samples;
            const gr_complex* chirp2 = samples + d_samples_per_symbol;
            float magsq_chirp1[window];
            float magsq_chirp2[window];
            float energy_chirp1 = 0;
            float energy_chirp2 = 0;
            float autocorr = 0;
            gr_complex dot_product;

            volk_32fc_x2_conjugate_dot_prod_32fc(&dot_product, chirp1, chirp2, window);
            volk_32fc_magnitude_squared_32f(magsq_chirp1, chirp1, window);
            volk_32fc_magnitude_squared_32f(magsq_chirp2, chirp2, window);
            volk_32f_accumulator_s32f(&energy_chirp1, magsq_chirp1, window);
            volk_32f_accumulator_s32f(&energy_chirp2, magsq_chirp2, window);


            // When using implicit mode, stop when energy is halved.
            d_energy_threshold = energy_chirp2 / 2.0f;

            // For calculating the SNR later on
            d_pwr_queue.push_back(energy_chirp1 / d_samples_per_symbol);

            // Autocorr value
            autocorr = abs(dot_product / gr_complex(sqrt(energy_chirp1 * energy_chirp2), 0));

        /*if(autocorr>= 0.90f)//lx
        {
        samples_to_file("/home/lx/decode_lora/preamble1.bin", chirp1, window, sizeof(gr_complex));
        samples_to_file("/home/lx/decode_lora/preamble2.bin", chirp2, window, sizeof(gr_complex));
        std::cout<<"store data"<<std::endl;
         }*/

            return autocorr;
        }

        float decoder_impl::determine_energy(const gr_complex *samples) {
            float magsq_chirp[d_samples_per_symbol];
            float energy_chirp = 0;
            volk_32fc_magnitude_squared_32f(magsq_chirp, samples, d_samples_per_symbol);
            volk_32f_accumulator_s32f(&energy_chirp, magsq_chirp, d_samples_per_symbol);

            return energy_chirp;
        }
        uint32_t decoder_impl::instantaneous_frequency_haha(const gr_complex *in_samples, float *out_ifreq, const uint32_t window){
            instantaneous_frequency(in_samples,out_ifreq, window);
            ifreq_temp.resize(window);
            float ifreq_max=0;
            //float yi=0;
            //float ifreq_value=0;
            for(uint32_t i=1;i<window;i++){
                ifreq_temp[i]=out_ifreq[i-1]-out_ifreq[i];
                if(ifreq_temp[i]>ifreq_max){
                    ifreq_max=ifreq_temp[i];
                }
            }
            
            float d_max=ifreq_max;
            int num=25;
            int k=0;
            int count=0;
            while (k<num&&count<5){
                k=0;
                d_max=d_max/2.0;
                for (uint32_t i=1;i<window;i++){
                    if (ifreq_temp[i]>d_max){
                          k=k+1;
                          if (k>num){
                              break;
                          }
                    }
                }
                if (k<num){
                    for (uint32_t i=1;i<window;i++){
                        if (ifreq_temp[i]>d_max){
                             ifreq_temp[i]= d_max;
                        }
                    }
                }
                count=count+1;
            }
            for (uint32_t i=1;i<window;i++){
                if (ifreq_temp[i]<d_max*(-1.0)){
                     ifreq_temp[i]= -d_max;
                }
            }
            
           /*for(uint32_t i=0;i<window;i++){
               ifreq_value=(float)ifreq_temp[i]-2.0/10.0;
               if(ifreq_value>0){
                   ifreq_temp[i]=0;}
               ifreq_value=(float)ifreq_temp[i]+2.0/10.0;
               if(ifreq_value<0){
                    ifreq_temp[i]=0;}
                //std::cout<<i<<" "<<ifreq_temp[i]<<" "<<ifreq_value<<std::endl;
            }
            
            for(uint32_t i=0;i<=window;i++){
                //std::cout<<i<<" "<<ifreq_temp[i]<<std::endl;
               if ( ifreq_temp[i]>ifreq_max){
                   ifreq_max=ifreq_temp[i];
                   yi=i;
                }
            }//lx*/
            //std::cout<<"2.5: "<<ifreq_max<<" "<<yi<<std::endl;
            
            float temp_avg=d_max/3;
            uint32_t max_gradient_count=0;
            uint32_t max_gradient_dexs[3]={0};
            int dex=0;
            bool flag_high=false;
            int flag_count=0;
            float max_gradient_temp=0;
            //std::cout<<"3: "<<temp_avg<<std::endl;
            
            for (uint32_t i=1;i<window;i++){
                if (ifreq_temp[i]>temp_avg){
                    flag_count=0;
                    if (flag_high==0){
                       flag_high=1;
                    }
                    if (ifreq_temp[i]>max_gradient_temp){
                        max_gradient_count=i;
                        max_gradient_temp=ifreq_temp[i];
                    } 
                }
                else{
                    if (flag_high==1){
                       flag_count=flag_count+1;}
                    if (flag_count==500){
                        flag_high=0;
                        flag_count=0;
                        max_gradient_temp=0;
                        max_gradient_dexs[dex]=max_gradient_count;
                        //std::cout<<"3.5: "<<(float)max_gradient_count<<" "<<dex<<" "<<ifreq_temp[max_gradient_count]<<std::endl;
                        dex=dex+1;
                        if(dex==2) break;
                        }
                    }
            }
            std::cout<<(float)max_gradient_dexs[0]<<" "<<(float)max_gradient_dexs[1]<<std::endl;
            uint32_t max_gradient_dex=0;
            if (dex>1){
                max_gradient_dex=max_gradient_dexs[1]-max_gradient_dexs[0];
                /*if(max_gradient_dex<max_gradient_dexs[0]){
                    max_gradient_dex=max_gradient_dexs[0];
                }
                else{
                    if(max_gradient_dexs[0]>800){
                        max_gradient_dex=(max_gradient_dexs[0]+max_gradient_dex)/2.0;                        
                    }
                }*/
                corr_len=max_gradient_dex;
                //std::cout<<(float)max_gradient_dexs[1]<<" "<<(float)max_gradient_dexs[0]<<std::endl;
            }
            else{
                max_gradient_dex=max_gradient_dexs[0];
                corr_len=max_gradient_dex;
                //std::cout<<(float)max_gradient_dexs[1]<<std::endl;
            }
            //std::cout<<std::endl;
            return max_gradient_dex;
        }
        
        int decoder_impl::dete_sf_signal(const gr_complex *samples) {            
            uint32_t j=0;
            int SF=0;
            const gr_complex* sample1 = samples;
            float magsq_sample1[numbles_for_SF];
            float temp=0;
            //std::cout<<"1 "<<std::endl;
            volk_32fc_magnitude_squared_32f(magsq_sample1, sample1, numbles_for_SF);
            //std::cout<<"2 "<<std::endl;
            for(uint32_t i=0;i<numbles_for_SF;i++){                     //find max
                magsq_sample1[i]=std::sqrt(magsq_sample1[i]);
                if(magsq_sample1[i]>temp){
                    temp=magsq_sample1[i];
                }
            }//std::cout<<"3 "<<std::endl;
            float limit=temp/4.0;
            gr_complex sample_temp [numbles_for_SF];
            
            for(uint32_t i=0; i<numbles_for_SF;i++){                        //delect no useful data
                  if(magsq_sample1[i]>limit){
                      sample_temp[j]=sample1[i];
                      j=j+1;
                    }
            }
            //std::cout<<"4"<<std::endl;
            out_sample.resize(j);
            out_sample_d4.resize(j/4);
            out_ifreq.resize(j);
            out_sample_len=j;
            int sample_count=0;
            uint32_t sample_dex=0;
            for(uint32_t i=0;i<j;i++){
                out_sample[i]=sample_temp[i];
                if(sample_count==4){
                    out_sample_d4[sample_dex]=sample_temp[i];
                    sample_count=0;
                    sample_dex=sample_dex+1;
                }
                sample_count=sample_count+1;
                out_ifreq[i]=0;
            }
            //std::cout<<"1"<<std::endl;
            uint32_t count=instantaneous_frequency_haha(&out_sample[0], &out_ifreq[0], j-1);
            //std::cout<<"2: "<<float(count)<<std::endl;
            if(count>500&&count<1500){
                SF=7;}
            else if (count>1600&&count<2500){
                SF=8; }
            else if (count>3000&&count<5000){
                SF=9;}
            else if (count>7500&&count<8500){
                SF=10;}
            return SF;
        }

        void decoder_impl::determine_snr() {
            if(d_pwr_queue.size() >= 2) {
                float pwr_noise = d_pwr_queue[0];
                float pwr_signal = d_pwr_queue[d_pwr_queue.size()-1];
                d_snr = pwr_signal / pwr_noise;
            }
        }

        float decoder_impl::detect_downchirp(const gr_complex *samples, const uint32_t window) {
            float samples_ifreq[window];
            instantaneous_frequency(samples, samples_ifreq, window);

            return cross_correlate_ifreq(samples_ifreq, d_downchirp_ifreq, window - 1u);
        }

        float decoder_impl::detect_upchirp(const gr_complex *samples, const uint32_t window, int32_t *index) {
            float samples_ifreq[window*2];
            instantaneous_frequency(samples, samples_ifreq, window*2);

            return sliding_norm_cross_correlate_upchirp(samples_ifreq, window, index);
        }

        float decoder_impl::sliding_norm_cross_correlate_upchirp(const float *samples_ifreq, const uint32_t window, int32_t *index) {
             float max_correlation = 0;

             // Cross correlate
             for (uint32_t i = 0; i < window; i++) {
                 const float max_corr = cross_correlate_ifreq_fast(samples_ifreq + i, &d_upchirp_ifreq[0], window - 1u);

                 if (max_corr > max_correlation) {
                     *index = i;
                     max_correlation = max_corr;
                 }
             }

             return max_correlation;
         }
        
        float decoder_impl::correlate_detSFBW_lx(const gr_complex *samples,  const uint32_t window) {
             float max_correlation = 0;
               //std::cout<<"1"<<std::endl;
             // Cross correlate
             for (uint32_t i = 0; i < window-d_upchirplx_len; i++) {
                 const float max_corr = cross_correlate_fast(samples+i, &d_upchirplx[0], d_upchirplx_len-1);
                  //std::cout<<float(i)<<std::endl;
                 if (max_corr > max_correlation) {
                     max_correlation = max_corr;
                 }
             }

             return max_correlation;
         }
        float decoder_impl::correlate_dete(const gr_complex *input,float auto_corr_D,int *para_dex) {
             float dete_value[numbers_para];
            
             /*float corr_value71=xcorr(&d_upchirp71[0], &input[0], NULL, upchirp71_len,numbles_for_SF);
             dete_value[0]=10*corr_value71/std::sqrt(auto_corr_D*auto_corr71);*/
            
             float corr_value72=xcorr(&d_upchirp72[0], &input[0], NULL, upchirp72_len,numbles_for_SF);
             dete_value[1]=10*corr_value72/std::sqrt(auto_corr_D*auto_corr72);
            
             float corr_value81=xcorr(&d_upchirp81[0], &input[0], NULL, upchirp81_len,numbles_for_SF);
             dete_value[2]=10*corr_value81/std::sqrt(auto_corr_D*auto_corr81);
            
             /*float corr_value82=xcorr(&d_upchirp82[0], &input[0], NULL, upchirp82_len,numbles_for_SF);
             dete_value[3]=10*corr_value82/std::sqrt(auto_corr_D*auto_corr82);
            
             float corr_value85=xcorr(&d_upchirp85[0], &input[0], NULL, upchirp85_len,numbles_for_SF);
             dete_value[4]=10*corr_value85/std::sqrt(auto_corr_D*auto_corr85);
            
             float corr_value91=xcorr(&d_upchirp91[0], &input[0], NULL, upchirp91_len,numbles_for_SF);
             dete_value[5]=10*corr_value91/std::sqrt(auto_corr_D*auto_corr91);
            
             float corr_value92=xcorr(&d_upchirp92[0], &input[0], NULL, upchirp92_len,numbles_for_SF);
             dete_value[6]=10*corr_value92/std::sqrt(auto_corr_D*auto_corr92);
            
             float corr_value95=xcorr(&d_upchirp95[0], &input[0], NULL, upchirp95_len,numbles_for_SF);
             dete_value[7]=10*corr_value95/std::sqrt(auto_corr_D*auto_corr95);
            
             float corr_value102=xcorr(&d_upchirp102[0], &input[0], NULL, upchirp102_len,numbles_for_SF);
             dete_value[8]=10*corr_value102/std::sqrt(auto_corr_D*auto_corr102);
            
             float corr_value105=xcorr(&d_upchirp105[0], &input[0], NULL, upchirp105_len,numbles_for_SF);
             dete_value[9]=10*corr_value105/std::sqrt(auto_corr_D*auto_corr105);*/
            
             float max_correlation=0;
             for (int i=0;i<numbers_para;i++){
                 if(dete_value[i]>max_correlation){
                     max_correlation=dete_value[i];
                     *para_dex=i;
                 }
             }
            
             return max_correlation;
         }

        float decoder_impl::stddev(const float *values, const uint32_t len, const float mean) {
            float variance = 0.0f;

            for (uint32_t i = 0u; i < len; i++) {
                const float temp = values[i] - mean;
                variance += temp * temp;
            }

            variance /= (float)len;
            return std::sqrt(variance);
        }

        /**
         *  Currently unstable due to center frequency offset.
         */
        uint32_t decoder_impl::get_shift_fft(const gr_complex *samples) {
            float fft_mag[d_number_of_bins];

            samples_to_file("/tmp/data", &samples[0], d_samples_per_symbol, sizeof(gr_complex));

            // Multiply with ideal downchirp
            for (uint32_t i = 0u; i < d_samples_per_symbol; i++) {
                d_mult_hf[i] = samples[i] * d_downchirp[i];
            }

            samples_to_file("/tmp/mult", &d_mult_hf[0], d_samples_per_symbol, sizeof(gr_complex));

            // Perform FFT
            fft_execute(d_q);

            // Decimate. Note: assumes fft size is multiple of decimation factor and number of bins is even
            // This decimation should be identical to numpy's approach
            const uint32_t N = d_number_of_bins;
            memcpy(&d_tmp[0],               &d_fft[0],                                     (N + 1u) / 2u * sizeof(gr_complex));
            memcpy(&d_tmp[ (N + 1u) / 2u ], &d_fft[d_samples_per_symbol - (N / 2u)],        N / 2u * sizeof(gr_complex));
            d_tmp[N / 2u] += d_fft[N / 2u];

            // Get magnitude
            for (uint32_t i = 0u; i < d_number_of_bins; i++) {
                fft_mag[i] = std::abs(d_tmp[i]);
            }

            samples_to_file("/tmp/fft", &d_tmp[0], d_number_of_bins, sizeof(gr_complex));

            fft_execute(d_qr); // For debugging
            samples_to_file("/tmp/resampled", &d_mult_hf[0], d_number_of_bins, sizeof(gr_complex));

            // Return argmax here
            return (std::max_element(fft_mag, fft_mag + d_number_of_bins) - fft_mag);
        }

        uint32_t decoder_impl::max_frequency_gradient_idx(const gr_complex *samples) {
            float samples_ifreq[d_samples_per_symbol];
            float samples_ifreq_avg[d_number_of_bins];

            samples_to_file("/tmp/data", &samples[0], d_samples_per_symbol, sizeof(gr_complex));

            instantaneous_frequency(samples, samples_ifreq, d_samples_per_symbol);

            for(uint32_t i = 0; i < d_number_of_bins; i++) {
                volk_32f_accumulator_s32f(&samples_ifreq_avg[i], &samples_ifreq[i*d_decim_factor], d_decim_factor);
                samples_ifreq_avg[i] /= d_decim_factor;
            }

            float max_gradient = 0.1f;
            float gradient = 0.0f;
            uint32_t max_index = 0;
            for (uint32_t i = 1u; i < d_number_of_bins; i++) {
                gradient = samples_ifreq_avg[i - 1] - samples_ifreq_avg[i];
                if (gradient > max_gradient) {
                    max_gradient = gradient;
                    max_index = i+1;
                }
            }

            return (d_number_of_bins - max_index) % d_number_of_bins;
        }

        bool decoder_impl::demodulate(const gr_complex *samples, const bool reduced_rate) {
            // DBGR_TIME_MEASUREMENT_TO_FILE("SFxx_method");

            // DBGR_START_TIME_MEASUREMENT(false, "only");

            uint32_t bin_idx = max_frequency_gradient_idx(samples);
            //uint32_t bin_idx = get_shift_fft(samples);
            if(d_enable_fine_sync){
                fine_sync(samples, bin_idx, std::max(d_decim_factor / 4u, 2u));
            //std::cout<<"demodulate d_fine_sync: "<<d_fine_sync<<std::endl;
        }

            // DBGR_INTERMEDIATE_TIME_MEASUREMENT();

            // Header has additional redundancy
            if (reduced_rate || d_sf > 10) {
                bin_idx = std::lround(bin_idx / 4.0f) % d_number_of_bins_hdr;
            }

            // Decode (actually gray encode) the bin to get the symbol value
            const uint32_t word = bin_idx ^ (bin_idx >> 1u);

            #ifdef DEBUG
                d_debug << gr::lora::to_bin(word, reduced_rate ? d_sf - 2u : d_sf) << " " << word << " (bin " << bin_idx << ")"  << std::endl;
            #endif
            d_words.push_back(word);

            // Look for 4+cr symbols and stop
            if (d_words.size() == (4u + d_phdr.cr)) {
                // Deinterleave
                deinterleave((reduced_rate || d_sf > 10) ? d_sf - 2u : d_sf);

                return true; // Signal that a block is ready for decoding
            }

            return false; // We need more words in order to decode a block
        }

        /**
         *  Correct the interleaving by extracting each column of bits after rotating to the left.
         *  <br/>(The words were interleaved diagonally, by rotating we make them straight into columns.)
         */
        void decoder_impl::deinterleave(const uint32_t ppm) {
            const uint32_t bits_per_word = d_words.size();
            const uint32_t offset_start  = ppm - 1u;

            std::vector<uint8_t> words_deinterleaved(ppm, 0u);

            if (bits_per_word > 8u) {
                // Not sure if this can ever occur. It would imply coding rate high than 4/8 e.g. 4/9.
                std::cerr << "[LoRa Decoder] WARNING : Deinterleaver: More than 8 bits per word. uint8_t will not be sufficient!\nBytes need to be stored in intermediate array and then packed into words_deinterleaved!" << std::endl;
                exit(1);
            }

            for (uint32_t i = 0u; i < bits_per_word; i++) {
                const uint32_t word = gr::lora::rotl(d_words[i], i, ppm);

                for (uint32_t j = (1u << offset_start), x = offset_start; j; j >>= 1u, x--) {
                    words_deinterleaved[x] |= !!(word & j) << i;
                }
            }

            #ifdef DEBUG
                print_interleave_matrix(d_debug, d_words, ppm);
                print_vector_bin(d_debug, words_deinterleaved, "D", sizeof(uint8_t) * 8u);
            #endif

            // Add to demodulated data
            d_demodulated.insert(d_demodulated.end(), words_deinterleaved.begin(), words_deinterleaved.end());

            // Cleanup
            d_words.clear();
        }

        void decoder_impl::decode(const bool is_header) {
            static const uint8_t shuffle_pattern[] = {5, 0, 1, 2, 4, 3, 6, 7};

            // For determining shuffle pattern
            //if (!is_header)
            //    values_to_file("/tmp/before_deshuffle", &d_demodulated[0], d_demodulated.size(), 8);

            deshuffle(shuffle_pattern, is_header);

            // For determining whitening sequence
            //if (!is_header)
            //    values_to_file("/tmp/after_deshuffle", &d_words_deshuffled[0], d_words_deshuffled.size(), 8);

            dewhiten(is_header ? gr::lora::prng_header : d_whitening_sequence);

            //if (!is_header)
            //    values_to_file("/tmp/after_dewhiten", &d_words_dewhitened[0], d_words_dewhitened.size(), 8);

            hamming_decode(is_header);
        }

        void decoder_impl::msg_lora_frame(void) {
            uint32_t len = sizeof(loratap_header_t) + sizeof(loraphy_header_t) + d_payload_length;
            uint32_t offset = 0;
            uint8_t buffer[len];
            loratap_header_t loratap_header;

            memset(buffer, 0, sizeof(uint8_t) * len);
            memset(&loratap_header, 0, sizeof(loratap_header));

            loratap_header.rssi.snr = (uint8_t)(10.0f * log10(d_snr) + 0.5);

            offset = gr::lora::build_packet(buffer, offset, &loratap_header, sizeof(loratap_header_t));
            offset = gr::lora::build_packet(buffer, offset, &d_phdr, sizeof(loraphy_header_t));
            offset = gr::lora::build_packet(buffer, offset, &d_decoded[0], d_payload_length);
            if(offset != len) {
                std::cerr << "decoder_impl::msg_lora_frame: invalid write" << std::endl;
                exit(1);
            }

            pmt::pmt_t payload_blob = pmt::make_blob(buffer, sizeof(uint8_t)*len);
            message_port_pub(pmt::mp("frames"), payload_blob);
        }

        void decoder_impl::deshuffle(const uint8_t *shuffle_pattern, const bool is_header) {
            const uint32_t to_decode = is_header ? 5u : d_demodulated.size();
            const uint32_t len       = sizeof(shuffle_pattern) / sizeof(uint8_t);
            uint8_t result;

            for (uint32_t i = 0u; i < to_decode; i++) {
                result = 0u;

                for (uint32_t j = 0u; j < len; j++) {
                    result |= !!(d_demodulated[i] & (1u << shuffle_pattern[j])) << j;
                }

                d_words_deshuffled.push_back(result);
            }

            #ifdef DEBUG
                print_vector_bin(d_debug, d_words_deshuffled, "S", sizeof(uint8_t)*8);
            #endif

            // We're done with these words
            if (is_header){
                d_demodulated.erase(d_demodulated.begin(), d_demodulated.begin() + 5u);
                d_words_deshuffled.push_back(0);
            } else {
                d_demodulated.clear();
            }
        }

        void decoder_impl::dewhiten(const uint8_t *prng) {
            const uint32_t len = d_words_deshuffled.size();
            for (uint32_t i = 0u; i < len; i++) {
                uint8_t xor_b = d_words_deshuffled[i] ^ prng[i];
                d_words_dewhitened.push_back(xor_b);
            }

            #ifdef DEBUG
                print_vector_bin(d_debug, d_words_dewhitened, "W", sizeof(uint8_t) * 8);
            #endif

            d_words_deshuffled.clear();
        }

        void decoder_impl::hamming_decode(bool is_header) {
            switch(d_phdr.cr) {
                case 4: case 3: { // Hamming(8,4) or Hamming(7,4)
                    //hamming_decode_soft(is_header);
                    uint32_t n = ceil(d_words_dewhitened.size() * 4.0f / (4.0f + d_phdr.cr));
                    uint8_t decoded[n];

                    fec_decode(d_h48_fec, n, &d_words_dewhitened[0], decoded);
                    if(!is_header)
                        swap_nibbles(decoded, n);
                    d_decoded.assign(decoded, decoded+n);
                    break;
                }
                case 2: case 1: { // Hamming(6,4) or Hamming(5,4)
                    // TODO: Report parity error to the user
                    extract_data_only(is_header);
                    break;
                }
            }

            d_words_dewhitened.clear();
        }

        /**
         * Deprecated
         */
        void decoder_impl::hamming_decode_soft(bool is_header) {
            uint32_t len = d_words_dewhitened.size();
            for (uint32_t i = 0u; i < len; i += 2u) {
                const uint8_t d2 = (i + 1u < len) ? hamming_decode_soft_byte(d_words_dewhitened[i + 1u]) : 0u;
                const uint8_t d1 = hamming_decode_soft_byte(d_words_dewhitened[i]);

                if(is_header)
                    d_decoded.push_back((d1 << 4u) | d2);
                else
                    d_decoded.push_back((d2 << 4u) | d1);
            }
        }

        void decoder_impl::extract_data_only(bool is_header) {
            static const uint8_t data_indices[4] = {1, 2, 3, 5};
            uint32_t len = d_words_dewhitened.size();

            for (uint32_t i = 0u; i < len; i += 2u) {
                const uint8_t d2 = (i + 1u < len) ? select_bits(d_words_dewhitened[i + 1u], data_indices, 4u) & 0xFF : 0u;
                const uint8_t d1 = (select_bits(d_words_dewhitened[i], data_indices, 4u) & 0xFF);

                if(is_header)
                    d_decoded.push_back((d1 << 4u) | d2);
                else
                    d_decoded.push_back((d2 << 4u) | d1);
            }
        }

        /**
         *  Old method to determine CFO. Currently unused.
         */
        void decoder_impl::determine_cfo(const gr_complex *samples) {
            float iphase[d_samples_per_symbol];
            const float div = (float) d_samples_per_second / (2.0f * M_PI);

            // Determine instant phase
            instantaneous_phase(samples, iphase, d_samples_per_symbol);

            float sum = 0.0f;

            for (uint32_t i = 1u; i < d_samples_per_symbol; i++) {
                sum += (float)((iphase[i] - iphase[i - 1u]) * div);
            }

            d_cfo_estimation = sum / (float)(d_samples_per_symbol - 1u);
        }

        /**
         * New method to determine CFO.
         */
        float decoder_impl::experimental_determine_cfo(const gr_complex *samples, uint32_t window) {
            gr_complex mult[window];
            float mult_ifreq[window];

            volk_32fc_x2_multiply_32fc(mult, samples, &d_downchirp[0], window);
            instantaneous_frequency(mult, mult_ifreq, window);

            return mult_ifreq[256] / (2.0 * M_PI) * d_samples_per_second;
        }

        int decoder_impl::work(int noutput_items,
                               gr_vector_const_void_star& input_items,
                               gr_vector_void_star&       output_items) {
            (void) noutput_items;
            (void) output_items;
            //std::cout<<"in"<<std::endl;
            const gr_complex *input     = (gr_complex *) input_items[0];
            //const gr_complex *raw_input = (gr_complex *) input_items[1]; // Input bypassed by low pass filter

            //d_fine_sync = 0; // Always reset fine sync
            
            //uint8_t ploayload[]={0x09, 0x30, 0x60, 0x00, 0x00, 0x01, 0x00, 0x00, 0xaf, 0x80, 0x07, 0x02, 0x8c, 0x1e,};
            
            //static int detect_count=0;
            //static uint32_t BW_LX=250000;
            //static int SF=0;
            //static int lxcount=1;
            //float corr_value1=0;
            //float corr_value2=0;
            //float corr_value3=0;
            //uint8_t sf_BW_temp[2]={0,0};
            
            //if((resetconfig==1)&&(comeback_flag==0)){
            //    set_init(d_samples_per_second, BW_LX, SF, d_implicit, d_phdr.cr, d_phdr.has_mac_crc, d_reduced_rate, !d_enable_fine_sync);
            //    resetconfig=0;
            //}
            //std::cout<<signal_flag<<" "<<comeback_flag<<std::endl;
            /*if((signal_flag==0)&&comeback_flag){
            
            float energy=determine_energy(input);
                if(energy>1){
                    std::cout<<std::endl;
                    std::cout<<"energy: "<<energy<<std::endl;
                    int r = sigsetjmp(env,1);
                    if( r == 0){
                        signal(SIGSEGV, recvSignal);
                        //std::cout<<lxcount1<<std::endl;
                        //if(lxcount1==3){std::cout<<"jilu"<<std::endl;
                        //samples_to_file("/home/lx/decode_lora/data2.bin", &input[0], numbles_for_SF, sizeof(gr_complex));//}
                        //lxcount1=lxcount1+1;
                        //std::cout<<"2"<<std::endl;
                        d_sf_old= d_sf;
                        d_bw_old= d_bw;
                       
                        SF=dete_sf_signal(&input[0]); 
                        //std::cout<<"3"<<std::endl;
                        std::cout<<"SF: "<<SF<<std::endl;
                        if(SF!=0){
                            std::cout<<"jilu"<<std::endl;
                            samples_to_file("/home/lx/decode_lora/data2.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
                        }

                        if(SF!=0){
                            build_ideal_chirps_lx(2000000,125000, SF-1);
                            corr_value1=correlate_detSFBW_lx(&out_sample_d4[0], corr_len);

                            build_ideal_chirps_lx(2000000,250000, SF);
                            corr_value2=correlate_detSFBW_lx(&out_sample_d4[0], corr_len);

                            build_ideal_chirps_lx(2000000,500000, SF+1);
                            corr_value3=correlate_detSFBW_lx(&out_sample_d4[0], corr_len);

                            std::cout<<"1 corr_value: "<<corr_value1<<std::endl;
                            std::cout<<"2 corr_value: "<<corr_value2<<std::endl;
                            std::cout<<"3 corr_value: "<<corr_value3<<std::endl;

                            if(corr_value1>corr_value2){
                                if(corr_value1>corr_value3){
                                    SF=SF-1;
                                    BW_LX=125000;
                                }
                                else{
                                    SF=SF+1;
                                    BW_LX=500000;
                                }
                            }
                            else{
                                if(corr_value2>corr_value3){
                                    SF=SF;
                                    BW_LX=250000;
                                }
                                else{
                                    SF=SF+1;
                                    BW_LX=500000;
                                }
                            }
                            
                            
                            std::cout<<"SF: "<<SF<<" BW: "<<BW_LX/1000.0<<"KHz"<<std::endl;

                            if(BW_LX!=d_bw){
                                float BW=BW_LX/1000.0;
                                std::cout<<"BE_diff"<<std::endl;
                                comeback_flag=0;
                                pmt::pmt_t payload_bloblx = pmt::cons(pmt::intern(std::string("BW")), pmt::from_double(BW));
                                message_port_pub(pmt::mp("hahaout"), payload_bloblx);//lx

                            }
                            else{
                                comeback_flag=1;
                            }

                            if((SF!=d_sf)||(BW_LX!=d_bw)){
                                std::cout<<"SF_diff "<<std::endl;
                                set_init(d_samples_per_second, BW_LX, SF, d_implicit, d_phdr.cr, d_phdr.has_mac_crc, d_reduced_rate, !d_enable_fine_sync);
                            }
                            signal_flag=1; 
                        }
                        //std::cout<<"signal coming"<<std::endl;
                    }
                    else{
                        std::cout<<"no so many data!"<<std::endl;
                        consume_each(d_samples_per_symbol);
                    }
                }
                else{
                    consume_each(d_samples_per_symbol);
                }
                    
            }*/
            
            //if(signal_flag&&comeback_flag)
            gettimeofday(&dwStart,NULL); 
            
            static bool dete_noise=true;
            float auto_corr_D=xcorr(&input[0], &input[0], NULL,  numbles_for_SF,numbles_for_SF);
            //float corr_value81=xcorr(&d_upchirp81[0], &input[0], NULL, upchirp81_len,numbles_for_SF);
            //float dete_value=10*corr_value81/std::sqrt(auto_corr_D*auto_corr81);
            if(dete_noise){
                int para_dex=0;
                float dete_value=correlate_dete(&input[0],auto_corr_D,&para_dex);
                if(dete_value>1.5){
                    std::cout<<"            dete_value: "<<dete_value<<" para_dex: "<<para_dex<<std::endl;
                }
                 consume_each(numbles_for_SF);
            }
            gettimeofday(&dwEnd,NULL);  
            dwTime = 1000000*(dwEnd.tv_sec-dwStart.tv_sec)+(dwEnd.tv_usec-dwStart.tv_usec);  
            printf("P_T: %ld\n",dwTime); 
            
            /*if(comeback_flag)
            {  
                switch (d_state) {
                    case gr::lora::DecoderState::DETECT: {
                        float correlation = detect_preamble_autocorr(input, d_samples_per_symbol);
                        
                        //std::cout<<"DETECT: "<<correlation<<std::endl;
                        if (correlation >= 0.90f) {
                            determine_snr();
                            #ifdef DEBUG
                                d_debug << "Ca: " << correlation << std::endl;
                            #endif
                            d_corr_fails = 0u;
                            d_state = gr::lora::DecoderState::SYNC;
                            std::cout<<"start_SYNC"<<std::endl;
                            //detect_count=0;
                            break;
                        }
                        
                        consume_each(d_samples_per_symbol);
                        //samples_to_file("/home/lx/decode_lora/after_detect.bin",  &input[0], d_samples_per_symbol*4, sizeof(gr_complex));
                        

                        break;
                    }

                    case gr::lora::DecoderState::SYNC: {
                        int i = 0;
                        
                        detect_upchirp(input, d_samples_per_symbol, &i);
                        
                        samples_to_file("/tmp/detect",  &input[i], d_samples_per_symbol, sizeof(gr_complex));
                        
                        consume_each(i);
                        
                        samples_to_file("/home/lx/decode_lora/data_upchirp.bin",  &input[0], d_samples_per_symbol*5, sizeof(gr_complex));
                        
                        d_state = gr::lora::DecoderState::FIND_SFD;
                        //std::cout<<"start_FIND_SFD,i= "<<i<<std::endl;
                        break;
                    }

                    case gr::lora::DecoderState::FIND_SFD: {
                        const float c = detect_downchirp(input, d_samples_per_symbol);
                        #ifdef DEBUG
                            d_debug << "Cd: " << c << std::endl;
                        #endif

                        //std::cout<<"c: "<<c<<std::endl;           

                        if (c > 0.96f) {

                        //std::cout<<"downchirp"<<std::endl;

                            #ifdef DEBUG
                                d_debug << "SYNC: " << c << std::endl;
                            #endif
                            // Debug stuff
                            samples_to_file("/tmp/sync", input, d_samples_per_symbol, sizeof(gr_complex));

                            d_state = gr::lora::DecoderState::PAUSE;
                        } else {
                            if(c < -0.97f) {
                                fine_sync(input, d_number_of_bins-1, d_decim_factor * 4);

                                //std::cout<<"d_fine_sync: "<<d_fine_sync<<std::endl;
                                //std::cout<<"upchirp"<<std::endl;
                            } else {
                                d_corr_fails++;
                                //std::cout<<"noise"<<std::endl;
                                if(d_corr_fails==3){
                                    d_state = gr::lora::DecoderState::SYNC;
                                }                                
                            }

                            if (d_corr_fails > 4u) {
                                d_state = gr::lora::DecoderState::DETECT;
                                //set_sf(7);
                                //std::cout<<"out_FIND_SFD"<<std::endl;
  
                                signal_flag=0;
                                #ifdef DEBUG
                                    d_debug << "Lost sync" << std::endl;
                                #endif
                            }
                        }

                        consume_each((int32_t)d_samples_per_symbol+d_fine_sync);
                        break;
                    }

                    case gr::lora::DecoderState::PAUSE: {
                        if(d_implicit){
                            d_state = gr::lora::DecoderState::DECODE_PAYLOAD;
                            d_payload_symbols = 1;
                        } else {
                            d_state = gr::lora::DecoderState::DECODE_HEADER;
                        }
                        consume_each(d_samples_per_symbol + d_delay_after_sync);
                        break;
                    }

                    case gr::lora::DecoderState::DECODE_HEADER: {
                        d_phdr.cr = 4u;

                        if (demodulate(input, true)) {
                            decode(true);
                            gr::lora::print_vector_hex(std::cout, &d_decoded[0], d_decoded.size(), false);
                            memcpy(&d_phdr, &d_decoded[0], sizeof(loraphy_header_t));
                            if (d_phdr.cr > 4)
                                d_phdr.cr = 4;
                            d_decoded.clear();

                            d_payload_length = d_phdr.length + MAC_CRC_SIZE * d_phdr.has_mac_crc;
                            //d_phy_crc = SM(decoded[1], 4, 0xf0) | MS(decoded[2], 0xf0, 4);

                            // Calculate number of payload symbols needed
                            uint8_t redundancy = (d_sf > 10 ? 2 : 0);
                            const int symbols_per_block = d_phdr.cr + 4u;
                            const float bits_needed     = float(d_payload_length) * 8.0f;
                            const float symbols_needed  = bits_needed * (symbols_per_block / 4.0f) / float(d_sf - redundancy);
                            const int blocks_needed     = (int)std::ceil(symbols_needed / symbols_per_block);
                            d_payload_symbols     = blocks_needed * symbols_per_block;

                            #ifdef DEBUG
                                d_debug << "LEN: " << d_payload_length << " (" << d_payload_symbols << " symbols)" << std::endl;
                            #endif

                            d_state = gr::lora::DecoderState::DECODE_PAYLOAD;
                        }

                        consume_each((int32_t)d_samples_per_symbol+d_fine_sync);
                        break;
                    }

                    case gr::lora::DecoderState::DECODE_PAYLOAD: {
                        if (d_implicit && determine_energy(input) < d_energy_threshold) {
                            d_payload_symbols = 0;
                            //d_demodulated.erase(d_demodulated.begin(), d_demodulated.begin() + 7u); // Test for SF 8 with header
                            d_payload_length = (int32_t)(d_demodulated.size() / 2);
                        } else if (demodulate(input, d_implicit && d_reduced_rate)) {
                            if(!d_implicit)
                                d_payload_symbols -= (4u + d_phdr.cr);
                        }

                        if (d_payload_symbols <= 0) {
                            decode(false);
                            gr::lora::print_vector_hex(std::cout, &d_decoded[0], d_payload_length, true);
                            
                            sf_BW_temp[0]=d_sf;
                            if(d_bw==125000){
                                sf_BW_temp[1]=1;
                            }
                            else if(d_bw==250000){
                                sf_BW_temp[1]=2;
                            }
                            else if(d_bw==500000){
                                sf_BW_temp[1]=5;
                            }
                            
                            samples_to_file_add("/home/lx/decode_lora/record.txt",  &d_decoded[0],&sf_BW_temp[0],d_payload_length);
                            //lxcount=0;
                            //message_port_pub(pmt::mp("hahaout"), payload_bloblx);//lx
                            //std::cout<<"out1"<<std::endl;
                            msg_lora_frame();

                            d_state = gr::lora::DecoderState::DETECT;
                            d_decoded.clear();
                            d_words.clear();
                            d_words_dewhitened.clear();
                            d_words_deshuffled.clear();
                            d_demodulated.clear();
                            signal_flag=0;
                        }

                        consume_each((int32_t)d_samples_per_symbol+d_fine_sync);

                        break;
                    }

                    case gr::lora::DecoderState::STOP: {
                        consume_each(d_samples_per_symbol);
                        break;
                    }

                    default: {
                        std::cerr << "[LoRa Decoder] WARNING : No state! Shouldn't happen\n";
                        break;
                    }
                }
            }
            else
            {
                consume_each(d_samples_per_symbol);
            }*/
            //std::cout<<"out"<<std::endl;
            // DBGR_INTERMEDIATE_TIME_MEASUREMENT();

            // Tell runtime system how many output items we produced.
            /*if(lxcount==0){
                lxcount=1;
                std::cout<<"hehe"<<std::endl;
                float BW=500000/1000.0;
                comeback_flag=0;
                pmt::pmt_t payload_bloblx = pmt::cons(pmt::intern(std::string("BW")), pmt::from_double(BW));
                message_port_pub(pmt::mp("hahaout"), payload_bloblx);//lx
                set_init(d_samples_per_second, 500000, 9, d_implicit, d_phdr.cr, d_phdr.has_mac_crc, d_reduced_rate, !d_enable_fine_sync);
            }*/
            return 0;
        }

        void decoder_impl::set_sf(const uint8_t sf) {
            d_sflx=sf;
            std::cout<<"d_sflx_changed: "<<float(sf)<<std::endl;
            set_init(d_samples_per_second, d_bw, d_sflx, d_implicit, d_phdr.cr, d_phdr.has_mac_crc, d_reduced_rate, !d_enable_fine_sync);
        //(void) sf;
            //std::cerr << "[LoRa Decoder] WARNING : Setting the spreading factor during execution is currently not supported." << std::endl
            //          << "Nothing set, kept SF of " << d_sf << "." << std::endl;
        }

        void decoder_impl::set_samp_rate(const float samp_rate) {
            (void) samp_rate;
            std::cerr << "[LoRa Decoder] WARNING : Setting the sample rate during execution is currently not supported." << std::endl
                      << "Nothing set, kept SR of " << d_samples_per_second << "." << std::endl;
        }
    } /* namespace lora */
} /* namespace gr */
