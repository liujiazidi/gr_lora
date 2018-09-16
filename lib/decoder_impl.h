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

#ifndef INCLUDED_LORA_DECODER_IMPL_H
#define INCLUDED_LORA_DECODER_IMPL_H

#include <liquid/liquid.h>
#include "lora/decoder.h"
#include <string>
#include <vector>
#include <fstream>
#include <lora/debugger.h>
#include <volk/volk.h>
#include <lora/loraphy.h>
#include <boost/circular_buffer.hpp>


#define numbers_para 10

#include <gnuradio/uhd/usrp_source.h>

typedef boost::shared_ptr<gr::uhd::usrp_source> dev_sptr;


namespace gr {
    namespace lora {

        /**
         *  \brief  **DecoderState** : Each state the LoRa decoder can be in.
         */
        enum class DecoderState {
            DETECT,
            SYNC,
            FIND_SFD,
            PAUSE,
            DECODE_HEADER,
            DECODE_PAYLOAD,
            STOP
        };

        /**
         *  \brief  Return the DecoderState as string for debugging purposes.
         *
         *  \param  s
         *          The state to return to string.
         */
        /*static std::string state_to_string(DecoderState s) {
            static std::string DecoderStateLUT[] = { "DETECT", "SYNC", "FIND_SFD", "PAUSE", "DECODE_HEADER", "DECODE_PAYLOAD", "STOP" };
            return DecoderStateLUT[ (size_t)s ];
        }*/

        /**
         *  \brief  **LoRa Decoder**
         *          <br/>The main class for the LoRa decoder.
         *          Contains all variables and methods necessary for succesfully decoding LoRa PHY.
         *          <br/>Only the sample rate and spreading factor are needed.
         *          The other settings, like packet length and coding rate, are extracted from the (explicit) HDR.
         */
        class decoder_impl : public decoder {
            private:
                debugger                d_dbg;              ///< Debugger for plotting samples, printing output, etc.
                DecoderState            d_state;            ///< Holds the current state of the decoder (state machine).

                std::vector<gr_complex> d_downchirp;        ///< The complex ideal downchirp.
                std::vector<float>      d_downchirp_ifreq;  ///< The instantaneous frequency of the ideal downchirp.

                std::vector<gr_complex> d_upchirp;          ///< The complex ideal upchirp.
                std::vector<float>      d_upchirp_ifreq;    ///< The instantaneous frequency of the ideal upchirp.
                std::vector<float>      d_upchirp_ifreq_v;  ///< The instantaneous frequency of the ideal upchirp.

                std::vector<gr_complex> d_fft;              ///< Vector containing the FFT resuls.
                std::vector<gr_complex> d_mult_hf;          ///< Vector containing the FFT decimation.
                std::vector<gr_complex> d_tmp;              ///< Vector containing the FFT decimation.

                bool             d_implicit;                ///< Implicit header mode.
                bool             d_reduced_rate;            ///< Use reduced rate (only configurable in implicit header mode).
                uint8_t          d_sf;                      ///< The Spreading Factor.

                uint8_t          d_sflx;

                uint32_t         d_bw;                      ///< The receiver bandwidth (fixed to `125kHz`).
                loraphy_header_t d_phdr;                    ///< LoRa PHY header.
                uint16_t         d_mac_crc;                 ///< The MAC CRC.
                double           d_bits_per_second;         ///< Indicator of how many bits are transferred each second.
                uint32_t         d_delay_after_sync;        ///< The number of samples to skip in `DecoderState::PAUSE`.
                uint32_t         d_samples_per_second;      ///< The number of samples taken per second by GNU Radio.
                double           d_symbols_per_second;      ///< Indicator of how many symbols (read: chirps) are transferred each second.
                double           d_bits_per_symbol;         ///< The number of bits each of the symbols contain.
                uint32_t         d_samples_per_symbol;      ///< The number of samples in one symbol.
                double           d_period;                  ///< Period of the symbol.
                uint32_t         d_number_of_bins;          ///< Indicates in how many parts or bins a symbol is decimated, i.e. the max value to decode out of one payload symbol.
                uint32_t         d_number_of_bins_hdr;      ///< Indicates in how many parts or bins a HDR symbol is decimated, i.e. the max value to decode out of one HDR symbol.
                 int32_t         d_payload_symbols;         ///< The number of symbols needed to decode the payload. Calculated from an indicator in the HDR.
                uint32_t         d_payload_length;          ///< The number of words after decoding the HDR or payload. Calculated from an indicator in the HDR.
                uint32_t         d_corr_fails;              ///< Indicates how many times the correlation failed. After some tries, the state will revert to `DecoderState::DETECT`.
                float            d_energy_threshold;        ///< The absolute threshold to distinguish signal from noise.
                const uint8_t*   d_whitening_sequence;      ///< A pointer to the whitening sequence to be used in decoding. Determined by the SF in the ctor.
                float            d_snr;                     ///< Signal to noise ratio
                boost::circular_buffer<float> d_pwr_queue;  ///< Queue holding symbol power values

                std::vector<uint32_t> d_words;              ///< Vector containing the demodulated words.
                std::vector<uint8_t>  d_demodulated;        ///< Vector containing the words after deinterleaving.
                std::vector<uint8_t>  d_words_deshuffled;   ///< Vector containing the words after deshuffling.
                std::vector<uint8_t>  d_words_dewhitened;   ///< Vector containing the words after dewhitening.
                std::vector<uint8_t>  d_decoded;            ///< Vector containing the words after Hamming decode or the final decoded words.
            
                gr_complex * signala_ext72x;
                gr_complex * signalb_ext72x;
                gr_complex * out_shifted72x;
                gr_complex * outa72x;
                gr_complex * outb72x;
                gr_complex * out72x;
                gr_complex * result72x;
                gr_complex * outb_conj72x;
                fftplan pa72x;
                fftplan pb72x;
                fftplan px72x;
            
                gr_complex * signala_ext82x;
                gr_complex * signalb_ext82x;
                gr_complex * out_shifted82x;
                gr_complex * outa82x;
                gr_complex * outb82x;
                gr_complex * out82x;
                gr_complex * result82x;
                gr_complex * outa_conj82x;
                fftplan pa82x;
                fftplan pb82x;
                fftplan px82x;
                gr_complex * match_filter82_in;
                gr_complex * match_filter82_out;
                fftplan m82x;
            
                gr_complex * signala_time_intervalx;
                gr_complex * signalb_time_intervalx;
                gr_complex * outa_time_intervalx;
                gr_complex * outb_time_intervalx;
                gr_complex * out_time_intervalx;
                gr_complex * result_timex;
                gr_complex * outa_conj_timex;
                fftplan pa_time_intervalx;
                fftplan pb_time_intervalx;
                fftplan px_time_intervalx;
                 
                int start_payload;
                gr_complex * signala_ext92x;
                gr_complex * signalb_ext92x;
                gr_complex * out_shifted92x;
                gr_complex * outa92x;
                gr_complex * outb92x;
                gr_complex * out92x;
                gr_complex * result92x;
                gr_complex * outa_conj92x;
                fftplan pa92x;
                fftplan pb92x;
                fftplan px92x;
            
                gr_complex * signala_ext102x;
                gr_complex * signalb_ext102x;
                gr_complex * out_shifted102x;
                gr_complex * outa102x;
                gr_complex * outb102x;
                gr_complex * out102x;
                gr_complex * result102x;
                gr_complex * outb_conj102x;
                fftplan pa102x;
                fftplan pb102x;
                fftplan px102x;
            
                gr_complex * signala_extd;
                gr_complex * signalb_extd;
                gr_complex * out_shiftedd;
                gr_complex * outad;
                gr_complex * outbd;
                gr_complex * outd;
                gr_complex * resultd;
                gr_complex * outb_conjd;
                fftplan pad;
                fftplan pbd;
                fftplan pxd;

                std::ofstream d_debug_samples;              ///< Debug utputstream for complex values.
                std::ofstream d_debug;                      ///< Outputstream for the debug log.

                fftplan d_q;                                ///< The LiquidDSP::FFT_Plan.
                fftplan d_qr;                               ///< The LiquidDSP::FFT_Plan in reverse.
                fec     d_h48_fec;                          ///< LiquidDSP Hamming 4/8 FEC.
                gr_complex* p_map;

                uint32_t      d_decim_factor;               ///< The number of samples (data points) in each bin.
                float         d_cfo_estimation;             ///< An estimation for the current Center Frequency Offset.
                double        d_dt;                         ///< Indicates how fast the frequency changes in a symbol (chirp).
                bool    d_enable_fine_sync;                 ///< Enable drift correction
                int32_t d_fine_sync;                        ///< Amount of drift correction to apply for next symbol
                bool    signal_flag;                             ///lx
                std::vector<gr_complex>  out_sample;
                std::vector<gr_complex>  out_sample_d4;
                std::vector<float>  out_ifreq;
                uint32_t out_sample_len;
                std::vector<float>  ifreq_temp;
                uint32_t corr_len;
                std::vector<gr_complex> d_upchirp71;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp71;
                uint32_t upchirp71_len;
                float auto_corr71;
            
                std::vector<gr_complex> d_upchirp72;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp72;
                uint32_t upchirp72_len;
                float auto_corr72;
            
                std::vector<gr_complex> d_upchirp81;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp81;
                uint32_t upchirp81_len;
                float auto_corr81;
            
                std::vector<gr_complex> d_upchirp82;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp82;
                uint32_t upchirp82_len;
                float auto_corr82;
            
                std::vector<gr_complex> d_upchirp85;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp85;
                uint32_t upchirp85_len;
                float auto_corr85;
                
                std::vector<gr_complex> d_upchirp91;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp91;
                uint32_t upchirp91_len;
                float auto_corr91;
            
                std::vector<gr_complex> d_upchirp92;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp92;
                uint32_t upchirp92_len;
                float auto_corr92;
            
                std::vector<gr_complex> d_upchirp95;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp95;
                uint32_t upchirp95_len;
                float auto_corr95;
            
                std::vector<gr_complex> d_upchirp102;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp102;
                uint32_t upchirp102_len;
                float auto_corr102;
            
                std::vector<gr_complex> d_upchirp105;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_dnchirp105;
                uint32_t upchirp105_len;
                float auto_corr105;
            
                std::vector<gr_complex> d_upchirplx;        ///< The complex ideal downchirp.
                std::vector<gr_complex> d_datalx;        
                uint32_t d_upchirplx_len;
                std::vector<float>      d_upchirp_ifreqlx;    ///< The instantaneous frequency of the ideal upchirp.
                bool comeback_flag;
                bool resetconfig;
                pmt::pmt_t d_portha_de;
                
                gr_complex* upchirp_3;
                
                uint8_t          d_sf_old;                      ///< The Spreading Factor.
                uint32_t         d_bw_old; 
            
                int file_fathertox711[2];
                int file_fathertox71[2];
                int file_x71tofather[2];
                
                int file_fathertox721[2];
                int file_fathertox72[2];
                int file_x72tofather[2];
            
                int file_fathertox811[2];
                int file_fathertox81[2];
                int file_x81tofather[2];
                
                int file_fathertox821[2];
                int file_fathertox82[2];
                int file_x82tofather[2];
            
                int file_fathertox851[2];
                int file_fathertox85[2];
                int file_x85tofather[2];
            
                int file_fathertox911[2];
                int file_fathertox91[2];
                int file_x91tofather[2];
                
                int file_fathertox921[2];
                int file_fathertox92[2];
                int file_x92tofather[2];
            
                int file_fathertox951[2];
                int file_fathertox95[2];
                int file_x95tofather[2];
            
                int file_fathertox1021[2];
                int file_fathertox102[2];
                int file_x102tofather[2];
                
                int file_fathertox1051[2];
                int file_fathertox105[2];
                int file_x105tofather[2];
            
                float xcorr(const gr_complex * signala, const gr_complex * signalb, gr_complex * result, uint32_t Na,uint32_t Nb,bool flag,uint32_t* dex_i);
                float xcorr72x(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb,bool flag);
                float xcorr82x(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb,bool flag,uint32_t* dex_i);
                float xcorr92x(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb,bool flag,uint32_t* dex_i);
                float xcorr102x(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb);
                float xcorrd(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb);
                void match_filter(const gr_complex * signala, const gr_complex * signalb,const gr_complex * signalc,float* value, uint32_t value_num,uint32_t Na);
                float xcorr_mini_time_interval(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb);
            
                double find_mini_time_interval(gr_complex * signal);
            
                float lxdete_value[numbers_para];
            
                //USRP_ptr
                dev_sptr usrp ;
                /**
                 *  \message handle lx
                 */
                void handle_controlha_back(pmt::pmt_t msg);

                /**
                 *  \brief  TODO
                 */
                float cross_correlate_ifreq_fast(const float *samples_ifreq, const float *ideal_chirp, const uint32_t window);

                /**
                 *  \brief  TODO
                 */
                float cross_correlate_fast(const gr_complex* samples, const gr_complex* ideal_chirp, const uint32_t window);

                /**
                 *  \brief  TODO
                 */
                void fine_sync(const gr_complex* in_samples, uint32_t bin_idx, int32_t search_space);
                void fine_synclx(const gr_complex* in_samples, uint32_t bin_idx, int32_t search_space);

                /**
                 * \brief Schmidl-Cox autocorrelation approach for approximately detecting the preamble.
                 */
                float detect_preamble_autocorr(const gr_complex *samples, uint32_t window);
                
                
                /**
                 *  \brief  TODO
                 */
                float experimental_determine_cfo(const gr_complex *samples, uint32_t window);

                /**
                 *  \brief  Generate the ideal up- and downchirps.
                 */
                void build_ideal_chirps(void);
            
                /**
                 *  \brief  Generate the ideal up- and downchirps.  lx
                 */
                void build_ideal_chirps_lx(float samp_rate,uint32_t bandwidth, uint8_t sf);
            
                void build_ideal_chirps_sf_bw(float samp_rate,uint32_t bandwidth, uint8_t sf,gr_complex* d_upchirplx,gr_complex* d_dnchirplx);
                uint32_t ideal_chirps_num(float samp_rate,uint32_t bandwidth, uint8_t sf);
                /**
                 *  \brief  Debug method to dump the given complex array to the given file in binary format.
                 *
                 *  \param  path
                 *          The path to the file to dump to.
                 *  \param  v
                 *          The complex array to dump.
                 *  \param  length
                 *          Length of said array.
                 *  \param  elem_size
                 *          `sizeof` the data in the array.
                 */
                void samples_to_file(const std::string path, const gr_complex *v, const uint32_t length, const uint32_t elem_size);
                void samples_to_file_add(const std::string path, const uint8_t *v, const uint8_t *config,const uint32_t length);
                void float_values_to_file(const std::string path, float *v, uint32_t length, uint32_t elem_size);
                void time_to_file_add(const std::string path, double time_value );
                /**
                  *  \brief  Debug method to dump the given values array to a file in textual format.
                  *
                  *  \param  path
                  *          The path to the file to dump to.
                  *  \param  v
                  *          The values array to dump.
                  *  \param  length
                  *          Length of said array.
                  *  \param  ppm
                  *          PPM value of the data.
                  */
                void values_to_file(const std::string path, const unsigned char *v, const uint32_t length, const uint32_t ppm);

                /**
                 *  \brief  Write the given complex array to the debug outputstream.
                 *
                 *  \param  v
                 *          The complex array.
                 *  \param  length
                 *          Length of said complex array.
                 */
                void samples_debug(const gr_complex *v, const uint32_t length);

                /**
                 *  \brief  Correct the shift of the given symbol to match the ideal upchirp by sliding cross correlating.
                 *
                 *  \param  samples_ifreq
                 *          The symbol to shift.
                 *  \param  window
                 *          The window in which the symbol can be shifted (length of given sample array).
                 *  \param  index
                 *          The new start index in the window for the found upchirp.
                 *  \return Also return the correlation coefficient.
                 */
                float sliding_norm_cross_correlate_upchirp(const float *samples_ifreq, const uint32_t window, int32_t *index);
            
                float correlate_detSFBW_lx(const gr_complex *samples, const uint32_t window);
                float correlate_dete(const gr_complex *input,float auto_corr_D,int *para_dex);

                /**
                 *  \brief Base method to start downchirp correlation and return the correlation coefficient.
                 *
                 *  \param  samples
                 *          The complex array of samples to detect a downchirp in.
                 *  \param  window
                 *          Length of said sample.
                 */
                float detect_downchirp(const gr_complex *samples, const uint32_t window);

                /**
                 *  \brief  Base method to start upchirp detection by calling `sliding_norm_cross_correlate_upchirp`.
                 *          <br/>Sets up the instantaneous frequency of the given complex symbol.
                 *
                 *  \param  samples
                 *          The complex array of samples to detect an upchirp in.
                 *  \param  window
                 *          Length of said sample.
                 *  \param  index
                 *          The index to shift with so the upchirp is correctly synced inside its window.
                 *  \return Also return the correlation coefficient.
                 */
                float detect_upchirp(const gr_complex *samples, const uint32_t window, int32_t *index);

                /**
                 *  \brief  Returns the correlation coefficient when correlating the given complex symbols in the given window.
                 *
                 *  \param  samples_1
                 *          The first complex symbol to correlate with.
                 *  \param  samples_2
                 *          The second complex symbol to correlate with.
                 *  \param  window
                 *          The window in which to perform correlation.
                 */
                float cross_correlate(const gr_complex *samples_1, const gr_complex *samples_2, const uint32_t window);

                /**
                 *  \brief  Returns the correlation coefficient of a real signal.
                 *          See https://en.wikipedia.org/wiki/Cross-correlation#Normalized_cross-correlation.
                 *
                 *  \param  samples_ifreq
                 *          The instantaneous frequency of the symbol to correlate with.
                 *  \param  ideal_chirp
                 *          The vector containing the ideal chirp to correlate with.
                 *  \param  to_idx
                 *          Correlation end index.
                 */
                float cross_correlate_ifreq(const float *samples_ifreq, const std::vector<float>& ideal_chirp, const uint32_t to_idx);

                /**
                 *  \brief  Returns the index of the bin containing the frequency change by using FFT.
                 *
                 *  \param  samples
                 *          The complex symbol to analyse.
                 */
                uint32_t get_shift_fft(const gr_complex *samples);

                /**
                 *  \brief  Determine the center frequency offset in the given symbol.
                 *
                 *  \param  samples
                 *          The complex symbol to analyse.
                 */
                void determine_cfo(const gr_complex *samples);

                /**
                 *  \brief  Determine the energy of a symbol.
                 *
                 *  \param  samples
                 *          The complex symbol to analyse.
                 */
                float determine_energy(const gr_complex *samples);
                
                 /**
                 *  analysis sf of signal
                 */
                int dete_sf_signal(const gr_complex *samples);

                /**
                 *  \brief  Determine the SNR
                 */
                void determine_snr();

                /**
                 *  \brief  Returns the index of the bin containing the frequency change.
                 *
                 *  \param  samples
                 *          The complex symbol to analyze.
                 */
                uint32_t max_frequency_gradient_idx(const gr_complex *samples);

                /**
                 *  \brief  Demodulate the given symbol and return true if all expected symbols have been parsed.
                 *
                 *  \param  samples
                 *          The complex symbol to demodulate.
                 *  \param  is_header
                 *          Whether the demodulated words were from the HDR.
                 */
                bool demodulate(const gr_complex *samples, const bool is_header);
                bool demodulatelx(const gr_complex *samples, const bool is_header);

                /**
                 *  \brief  Deinterleave the raw demodulated words by reversing the interleave pattern.
                 *
                 *  \param  ppm
                 *          The number of words that zere interleaved. Depends on `SF`.
                 */
                void deinterleave(const uint32_t ppm);

                /**
                 *  \brief  The process of decoding the demodulated words to get the actual payload.
                 *          <br/>1. Deshuffle the words
                 *          <br/>2. Dewhiten the words
                 *          <br/>3. Hamming decoding
                 *
                 *  \param  is_header
                 *          Whether the demodulated words were from the HDR.
                 */
                void decode(const bool is_header);

                /**
                 *  \brief  Deshuffle the demodulated words by the given pattern.
                 *
                 *  \param  shuffle_pattern
                 *          The order in which the bits appear.
                 *  \param  is_header
                 *          Whether the demodulated words were from the HDR.
                 */
                void deshuffle(const uint8_t *shuffle_pattern, const bool is_header);

                /**
                 *  \brief  Dewhiten the deshuffled words by XORing with the whitening sequence.
                 *
                 *  \param  prng
                 *          The whitening sequence to XOR with.
                 */
                void dewhiten(const uint8_t *prng);

                /**
                 *  \brief  Use Hamming to decode the dewhitened words.
                 *          <br/>- CR 4 or 3: Hamming(8,4) or Hamming(7,4) with parity correction
                 *          <br/>- CR 2 or 1: Extract data only (can only find parity errors, not correct them)
                 *
                 *  \param  is_header
                 *          Decoding for the header?
                 */
                void hamming_decode(bool is_header);

                /**
                 *  \brief  Extract only the data in the given bytes.
                 *
                 *  \param  is_header
                 *          Decoding for the header?
                 */
                void extract_data_only(bool is_header);

                /**
                 *  \brief  Hamming(8,4) decoding by calling `hamming_decode_soft_byte` on each byte.
                 *          <BR>Each byte is decoded in pairs, the first one becoming the LSB nibble
                 *          <BR>and the second one the MSB nibble (if even; else just zeroes).
                 *
                 *  \param  is_header
                 *          Decoding for the header?
                 */
                void hamming_decode_soft(bool is_header);

                /**
                 *  \brief  Return the standard deviation for the given array.
                 *          <br/>Used for cross correlating.
                 *
                 *  \param  values
                 *          The array to calculate the standard deviation for.
                 *  \param  len
                 *          Length of said array.
                 *  \param  mean
                 *          The mean (average) of the values in the array.
                 */
                float stddev(const float *values, const uint32_t len, const float mean);

                /**
                 *  \brief  Calculate the instantaneous phase for the given complex symbol.
                 *
                 *  \param  in_samples
                 *          The complex array to calculate the instantaneous phase for.
                 *  \param  out_iphase
                 *          The output `float` array containing the instantaneous phase.
                 *  \param  window
                 *          The size of said arrays.
                 */
                inline void instantaneous_phase(const gr_complex *in_samples, float *out_iphase, const uint32_t window);

                /**
                 *  \brief  Calculate the instantaneous frequency for the given complex symbol.
                 *
                 *  \param  in_samples
                 *          The complex array to calculate the instantaneous frequency for.
                 *  \param  out_ifreq
                 *          The output `float` array containing the instantaneous frequency.
                 *  \param  window
                 *          The size of said arrays.
                 */
                inline void instantaneous_frequency(const gr_complex *in_samples, float *out_ifreq, const uint32_t window);
            
                uint32_t instantaneous_frequency_haha(const gr_complex *in_samples, float *out_ifreq, const uint32_t window);

                /**
                 *  \brief  TODO
                 */
                void msg_lora_frame(void);

                /**
                  *   \init
                  */
                void set_init(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, bool disable_drift_correction);

            public:
                /**
                 *  \brief  Default constructor.
                 *
                 *  \param  samp_rate
                 *          The sample rate of the input signal given to `work` later.
                 *  \param  sf
                 *          The expected spreqding factor.
                 */
                decoder_impl(float samp_rate, uint32_t bandwidth, uint8_t sf, bool implicit, uint8_t cr, bool crc, bool reduced_rate, bool disable_drift_correction);

                /**
                 *  Default destructor.
                 */
                ~decoder_impl();

                /**
                *   \brief  The main method called by GNU Radio to perform tasks on the given input.
                *
                *   \param  noutput_items
                *           The requested amoutn of output items.
                *   \param  input_items
                *           An array with samples to process.
                *   \param  output_items
                *           An array to return processed samples.
                *   \return Returns the number of output items generated.
                */
                int work(int noutput_items,
                         gr_vector_const_void_star& input_items,
                         gr_vector_void_star& output_items);

                /**
                 *  \brief  Set th current spreading factor.
                 *          <br/>**Currently not supported, restart GNU Radio with different settings instead.**
                 *  \param  sf
                 *          The new spreading factor.
                 */
                virtual void set_sf(const uint8_t sf);

                /**
                 *  \brief  Set the current sample rate.
                 *          <br/>**Currently not supported, restart GNU Radio with different settings instead.**
                 *
                 *  \param  samp_rate
                 *          The new sample rate.
                 */
                virtual void set_samp_rate(const float samp_rate);
            
                virtual void set_usrp_sptr(dev_sptr usrp_sprt);
        };
    } // namespace lora
} // namespace gr

#endif /* INCLUDED_LORA_DECODER_IMPL_H */
