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

#include <uhd/utils/paths.hpp>
#include <uhd/utils/thread.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/usrp_clock/multi_usrp_clock.hpp>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <complex>
#include <string>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <chrono>
#include <thread>

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
#include <unistd.h>
#include <sys/wait.h>
#include <sys/types.h>
#include <sys/mman.h>

#include <stdio.h> 
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h> 
#include <errno.h>

jmp_buf env;

#include "dbugr.hpp"
#define DEBUG 1
#define numbles_for_SF    7169//8404
#define SIG_RECVDATA    __SIGRTMIN+10
#define sample_lx 1000000
#define INPUT 0   
#define OUTPUT 1  

#define FALSElx -1
#define TRUElx 0

struct timeval dwStart;
struct timeval dwEnd;

struct termio
{ 
    unsigned short c_iflag;
    unsigned short c_oflag;
    unsigned short c_cflag; 
    unsigned short c_lflag; 
    unsigned char c_line; 
    unsigned char c_cc[NCCS]; 
};
int speed_arr[] = {B38400, B19200, B9600, B4800, B2400, B1200, B300,B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {38400, 19200, 9600, 4800, 2400, 1200, 300, 38400,19200, 9600, 4800, 2400, 1200, 300, };
void set_speed(int fd, int speed){
    int i;
    int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for ( i= 0; i < sizeof(speed_arr) / sizeof(int); i++) {
        if (speed == name_arr[i]) {
        tcflush(fd, TCIOFLUSH);
        cfsetispeed(&Opt, speed_arr[i]);
        cfsetospeed(&Opt, speed_arr[i]);
        status = tcsetattr(fd, TCSANOW, &Opt);
        if (status != 0) {
            perror("tcsetattr fd1");
            return;
        }
    tcflush(fd,TCIOFLUSH);
    }
    }
}

/**
*@brief 设置串口数据位，停止位和效验位
*@param fd 类型 int 打开的串口文件句柄
*@param databits 类型 int 数据位 取值 为 7 或者8
*@param stopbits 类型 int 停止位 取值为 1 或者2
*@param parity 类型 int 效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;
    if ( tcgetattr( fd,&options) != 0) {
        perror("SetupSerial 1");
        return(FALSElx);
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) /*设置数据位数*/
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data sizen"); 
            return (FALSElx);
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB; /* Clear parity enable */
            options.c_iflag &= ~INPCK; /* Enable parity checking */
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
            options.c_iflag |= INPCK; /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB; /* Enable parity */
            options.c_cflag &= ~PARODD; /* 转换为偶效验*/
            options.c_iflag |= INPCK; /* Disnable parity checking */
            break;
        case 'S':
        case 's': /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;
        default:
            fprintf(stderr,"Unsupported parityn");
            return (FALSElx);
    }
    /* 设置停止位*/
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported stop bitsn");
            return (FALSElx);
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSElx);
    }
    return (TRUElx);
}
int OpenDev(char *Dev)
{
    int fd = open( Dev, O_RDWR ); //| O_NOCTTY | O_NDELAY
    if (-1 == fd)
    {
    perror("Can't Open Serial Port");
    return -1;
    }
    else
    return fd;
}
static int time_point_count=2;
int write_uart(double TimePoint){
    int fd=0;
    int nread=0;
    char buff[40];
    sprintf(buff,"%d%c",time_point_count,'#');
    sprintf(buff,"%.9lf%c",TimePoint,'A');
    time_point_count=time_point_count+1;
    int len=0;
    for(int i=0;i<40;i++){
        if(buff[i]=='A'){
            len=i;
            break;
        }
    }
    char *dev = "/dev/ttyUSB1"; //串口二
    fd = OpenDev(dev);
    set_speed(fd,9600);
    if (set_Parity(fd,8,1,'N') == FALSElx) {
        printf("Set Parity Errorn");
        exit (0);
    }
    printf("Start!\n");
    //for(int i=0;i<10;i++) //循环读取数据
    //{
        nread = write(fd, buff,len);
        printf("nread=%d\n",nread);
    //}
    close(fd);
}


unsigned long dwTime=0;
pid_t child71, child72, child81, child82, child85, child91, child92, child95,child102, child105, child;


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
        
        void handler(int sig)
        {
            printf("get a sig,num is %d\n",sig);
            kill(child71,9);
            //kill(child72,9);
            /*kill(child81,9);
            kill(child82,9);
            kill(child85,9);
            kill(child91,9);*/
            /*kill(child92,9);
            kill(child95,9);
            kill(child102,9);
            kill(child105,9);*/
            signal(2,SIG_DFL);
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
            d_state = gr::lora::DecoderState::SYNC;

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

            usrp=NULL;
            set_init(samp_rate, bandwidth, sf, implicit, cr, crc, reduced_rate, disable_drift_correction);
                
            //std::cout<<"try1"<<std::endl;
            gr_complex result=0;
                
            resultd = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+numbles_for_SF-1));
            signala_extd = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+numbles_for_SF-1));
            signalb_extd = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+numbles_for_SF-1));
            outad = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+numbles_for_SF-1));
            outbd = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+numbles_for_SF-1));
            outd = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+numbles_for_SF-1));
            outb_conjd = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+numbles_for_SF-1));
            pad = fft_create_plan(numbles_for_SF+numbles_for_SF-1, &signala_extd[0], &outad[0], LIQUID_FFT_FORWARD, 0);
            pbd = fft_create_plan(numbles_for_SF+numbles_for_SF-1, &signalb_extd[0], &outbd[0], LIQUID_FFT_FORWARD, 0);
            pxd = fft_create_plan(numbles_for_SF+numbles_for_SF-1, &outd[0], &resultd[0], LIQUID_FFT_BACKWARD, 0); 
                
            upchirp71_len=ideal_chirps_num(sample_lx,125000,7);
            d_upchirp71.resize(upchirp71_len);
            d_dnchirp71.resize(upchirp71_len);
            build_ideal_chirps_sf_bw(sample_lx,125000,7,&d_upchirp71[0],&d_dnchirp71[0]);
                
            /*result71 = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp71_len+upchirp71_len-1));
            signala_ext71 = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp71_len+upchirp71_len-1));
            signalb_ext71 = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp71_len+upchirp71_len-1));
            outa71 = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp71_len+upchirp71_len-1));
            outb71 = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp71_len+upchirp71_len-1));
            out71 = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp71_len+upchirp71_len-1));
            outb_conj71 = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp71_len+upchirp71_len-1));
            pa71 = fft_create_plan(upchirp71_len+upchirp71_len-1, &signala_ext71[0], &outa71[0], LIQUID_FFT_FORWARD, 0);
            pb71 = fft_create_plan(upchirp71_len+upchirp71_len-1, &signalb_ext71[0], &outb71[0], LIQUID_FFT_FORWARD, 0);
            px71 = fft_create_plan(upchirp71_len+upchirp71_len-1, &out71[0], &result71[0], LIQUID_FFT_BACKWARD, 0); */
            
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp71[0], &d_upchirp71[0], upchirp71_len);//xcorr(&d_upchirp71[0], &d_upchirp71[0], NULL,  upchirp71_len,upchirp71_len);
            auto_corr71=abs(result);
                
            upchirp72_len=ideal_chirps_num(sample_lx,250000,7);
            d_upchirp72.resize(upchirp72_len);
            d_dnchirp72.resize(upchirp72_len);
            build_ideal_chirps_sf_bw(sample_lx,250000,7,&d_upchirp72[0],&d_dnchirp72[0]);
            
            result72x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp72_len-1));
            signala_ext72x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp72_len-1));
            signalb_ext72x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp72_len-1));
            outa72x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp72_len-1));
            outb72x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp72_len-1));
            out72x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp72_len-1));
            outb_conj72x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp72_len-1));
            pa72x = fft_create_plan(numbles_for_SF+upchirp72_len-1, &signala_ext72x[0], &outa72x[0], LIQUID_FFT_FORWARD, 0);
            pb72x = fft_create_plan(numbles_for_SF+upchirp72_len-1, &signalb_ext72x[0], &outb72x[0], LIQUID_FFT_FORWARD, 0);
            px72x = fft_create_plan(numbles_for_SF+upchirp72_len-1, &out72x[0], &result72x[0], LIQUID_FFT_BACKWARD, 0); 
                
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp72[0], &d_upchirp72[0], upchirp72_len);
            auto_corr72=abs(result);
                
            upchirp81_len=ideal_chirps_num(sample_lx,125000,8);
            d_upchirp81.resize(upchirp81_len);
            d_dnchirp81.resize(upchirp81_len);
            build_ideal_chirps_sf_bw(sample_lx,125000,8,&d_upchirp81[0],&d_dnchirp81[0]);
              
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp81[0], &d_upchirp81[0], upchirp81_len);
            auto_corr81=abs(result);
            //std::cout<<"auto_corr81: "<<auto_corr81<<std::endl;
                
            upchirp82_len=ideal_chirps_num(sample_lx,250000,8);
            d_upchirp82.resize(upchirp82_len);
            d_dnchirp82.resize(upchirp82_len);
            build_ideal_chirps_sf_bw(sample_lx,250000,8,&d_upchirp82[0],&d_dnchirp82[0]);
            
            result82x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp82_len-1));
            signala_ext82x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp82_len-1));
            signalb_ext82x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp82_len-1));
            outa82x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp82_len-1));
            outb82x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp82_len-1));
            out82x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp82_len-1));
            outa_conj82x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp82_len-1));
            pa82x = fft_create_plan(numbles_for_SF+upchirp82_len-1, &signala_ext82x[0], &outa82x[0], LIQUID_FFT_FORWARD, 0);
            pb82x = fft_create_plan(numbles_for_SF+upchirp82_len-1, &signalb_ext82x[0], &outb82x[0], LIQUID_FFT_FORWARD, 0);
            px82x = fft_create_plan(numbles_for_SF+upchirp82_len-1, &out82x[0], &result82x[0], LIQUID_FFT_BACKWARD, 0); 
                
            match_filter82_in = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp82_len));
            match_filter82_out = (gr_complex *) malloc(sizeof(gr_complex) * (upchirp82_len));
            m82x = fft_create_plan(upchirp82_len, &match_filter82_in[0], &match_filter82_out[0], LIQUID_FFT_FORWARD, 0); 
                
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp82[0], &d_upchirp82[0], upchirp82_len);
            auto_corr82=abs(result);
                
            upchirp85_len=ideal_chirps_num(sample_lx,500000,8);
            d_upchirp85.resize(upchirp85_len);
            d_dnchirp85.resize(upchirp85_len);
            build_ideal_chirps_sf_bw(sample_lx,500000,8,&d_upchirp85[0],&d_dnchirp85[0]);
                
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp85[0], &d_upchirp85[0], upchirp85_len);
            auto_corr85=abs(result);
                
            upchirp91_len=ideal_chirps_num(sample_lx,125000,9);
            d_upchirp91.resize(upchirp91_len);
            d_dnchirp91.resize(upchirp91_len);
            build_ideal_chirps_sf_bw(sample_lx,125000,9,&d_upchirp91[0],&d_dnchirp91[0]);
                
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp91[0], &d_upchirp91[0], upchirp91_len);
            auto_corr91=abs(result);
                
            upchirp92_len=ideal_chirps_num(sample_lx,250000,9);
            d_upchirp92.resize(upchirp92_len);
            d_dnchirp92.resize(upchirp92_len);
            build_ideal_chirps_sf_bw(sample_lx,250000,9,&d_upchirp92[0],&d_dnchirp92[0]);
            
            result92x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp92_len-1));
            signala_ext92x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp92_len-1));
            signalb_ext92x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp92_len-1));
            outa92x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp92_len-1));
            outb92x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp92_len-1));
            out92x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp92_len-1));
            outb_conj92x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp92_len-1));
            pa92x = fft_create_plan(numbles_for_SF+upchirp92_len-1, &signala_ext92x[0], &outa92x[0], LIQUID_FFT_FORWARD, 0);
            pb92x = fft_create_plan(numbles_for_SF+upchirp92_len-1, &signalb_ext92x[0], &outb92x[0], LIQUID_FFT_FORWARD, 0);
            px92x = fft_create_plan(numbles_for_SF+upchirp92_len-1, &out92x[0], &result92x[0], LIQUID_FFT_BACKWARD, 0); 
                
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp92[0], &d_upchirp92[0], upchirp92_len);
            auto_corr92=abs(result);
                
            upchirp95_len=ideal_chirps_num(sample_lx,500000,9);
            d_upchirp95.resize(upchirp95_len);
            d_dnchirp95.resize(upchirp95_len);
            build_ideal_chirps_sf_bw(sample_lx,500000,9,&d_upchirp95[0],&d_dnchirp95[0]);
                
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp95[0], &d_upchirp95[0], upchirp95_len);
            auto_corr95=abs(result);
                
            upchirp102_len=ideal_chirps_num(sample_lx,250000,10);
            d_upchirp102.resize(upchirp102_len);
            d_dnchirp102.resize(upchirp102_len);
            build_ideal_chirps_sf_bw(sample_lx,250000,10,&d_upchirp102[0],&d_dnchirp102[0]);
            
            /*result102x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp102_len-1));
            signala_ext102x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp102_len-1));
            signalb_ext102x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp102_len-1));
            outa102x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp102_len-1));
            outb102x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp102_len-1));
            out102x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp102_len-1));
            outb_conj102x = (gr_complex *) malloc(sizeof(gr_complex) * (numbles_for_SF+upchirp102_len-1));
            pa102x = fft_create_plan(numbles_for_SF+upchirp102_len-1, &signala_ext102x[0], &outa102x[0], LIQUID_FFT_FORWARD, 0);
            pb102x = fft_create_plan(numbles_for_SF+upchirp102_len-1, &signalb_ext102x[0], &outb102x[0], LIQUID_FFT_FORWARD, 0);
            px102x = fft_create_plan(numbles_for_SF+upchirp102_len-1, &out102x[0], &result102x[0], LIQUID_FFT_BACKWARD, 0); */
                
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp102[0], &d_upchirp102[0], upchirp102_len);
            auto_corr102=abs(result);
                
            upchirp105_len=ideal_chirps_num(sample_lx,500000,10);
            d_upchirp105.resize(upchirp105_len);
            d_dnchirp105.resize(upchirp105_len);
            build_ideal_chirps_sf_bw(sample_lx,500000,10,&d_upchirp105[0],&d_dnchirp105[0]);
                
            volk_32fc_x2_conjugate_dot_prod_32fc(&result, &d_upchirp105[0], &d_upchirp105[0], upchirp105_len);
            auto_corr105=abs(result);
                
            pipe(file_fathertox711);
            pipe(file_fathertox71);
            pipe(file_x71tofather);
                
            pipe(file_fathertox721);
            pipe(file_fathertox72);
            pipe(file_x72tofather); 
                
            pipe(file_fathertox811);
            pipe(file_fathertox81);
            pipe(file_x81tofather); 
            
            pipe(file_fathertox821);
            pipe(file_fathertox82);
            pipe(file_x82tofather); 
                
            pipe(file_fathertox851);
            pipe(file_fathertox85);
            pipe(file_x85tofather); 
                
            pipe(file_fathertox911);
            pipe(file_fathertox91);
            pipe(file_x91tofather);
                
            pipe(file_fathertox921);
            pipe(file_fathertox92);
            pipe(file_x92tofather); 
                
            pipe(file_fathertox951);
            pipe(file_fathertox95);
            pipe(file_x95tofather); 
            
            pipe(file_fathertox1021);
            pipe(file_fathertox102);
            pipe(file_x102tofather); 
                
            pipe(file_fathertox1051);
            pipe(file_fathertox105);
            pipe(file_x105tofather);
            //std::cout<<"try2 "<<std::endl;
            for (int i=0;i<numbers_para;i++){
                lxdete_value[i]=0;
            }
                
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
            set_output_multiple(numbles_for_SF);
            
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
            build_ideal_chirps_lx(16000000,125000, 7);
        
            result_timex = (gr_complex *) malloc(sizeof(gr_complex) * (d_upchirplx_len+d_upchirplx_len-1));
            signala_time_intervalx = (gr_complex *) malloc(sizeof(gr_complex) * (d_upchirplx_len+d_upchirplx_len-1));
            signalb_time_intervalx = (gr_complex *) malloc(sizeof(gr_complex) * (d_upchirplx_len+d_upchirplx_len-1));
            outa_time_intervalx = (gr_complex *) malloc(sizeof(gr_complex) * (d_upchirplx_len+d_upchirplx_len-1));
            outb_time_intervalx = (gr_complex *) malloc(sizeof(gr_complex) * (d_upchirplx_len+d_upchirplx_len-1));
            out_time_intervalx = (gr_complex *) malloc(sizeof(gr_complex) * (d_upchirplx_len+d_upchirplx_len-1));
            outa_conj_timex = (gr_complex *) malloc(sizeof(gr_complex) * (d_upchirplx_len+d_upchirplx_len-1));
            pa_time_intervalx = fft_create_plan(d_upchirplx_len+d_upchirplx_len-1, &signala_time_intervalx[0], &outa_time_intervalx[0], LIQUID_FFT_FORWARD, 0);
            pb_time_intervalx = fft_create_plan(d_upchirplx_len+d_upchirplx_len-1, &signalb_time_intervalx[0], &outb_time_intervalx[0], LIQUID_FFT_FORWARD, 0);
            px_time_intervalx = fft_create_plan(d_upchirplx_len+d_upchirplx_len-1, &out_time_intervalx[0], &result_timex[0], LIQUID_FFT_BACKWARD, 0); 
            
            //double dex=find_mini_time_interval(&d_upchirp[0]);
            //std::cout<<"Dex: "<<dex<<std::endl;
        
            // FFT decoding preparations
            d_fft.resize(d_samples_per_symbol);
            d_mult_hf.resize(d_samples_per_symbol);
            d_tmp.resize(d_number_of_bins);
            d_q  = fft_create_plan(d_samples_per_symbol, &d_mult_hf[0], &d_fft[0],     LIQUID_FFT_FORWARD, 0);
            d_qr = fft_create_plan(d_number_of_bins,     &d_tmp[0],     &d_mult_hf[0], LIQUID_FFT_BACKWARD, 0);

            // Hamming coding
            fec_scheme fs = LIQUID_FEC_HAMMING84;
            d_h48_fec = fec_create(fs, NULL);
            program_start_all_lx=true;
			time_uart=0.0;

    }
        
        double decoder_impl::find_mini_time_interval(const gr_complex * signal){
            d_datalx.resize(d_upchirplx_len);
            for(uint32_t i=0;i<d_samples_per_symbol;i++){
                d_datalx[i*16]=signal[i];
                for(uint32_t j=1;j<16;j++){
                    d_datalx[16*i+j]=0;
                }
            }
            double dex=xcorr_mini_time_interval(&d_upchirplx[0],&d_datalx[0],d_upchirplx_len,d_upchirplx_len);
            return dex-16384;//16384 is for 16*sample_rate(1M)
        }
        
        void decoder_impl::match_filter(const gr_complex * signala, const gr_complex * signalb,const gr_complex * signalc,float* value, uint32_t value_num,uint32_t Na=2)
        {
            float* p_value=(float *) malloc(sizeof(float) * (Na));
            float* p_sqrt=(float *) malloc(sizeof(float) * (Na));
            float value_sqrt=0;
            float max=0;
            int dex=0;
            float max_temp=0;
            bool max_start=false;
            int count=0;
            int max_count=0;
            bool max_error=false;
            float max_old=0;
            //std::cout<<(float)value_num<<" "<<(float)Na<<std::endl;
            for(uint32_t i=0;i<value_num;i++){
                //std::cout<<"i: "<<i<<std::endl;
                for(uint32_t j=0;j<Na;j++){
                    dex=i*Na+j;
                    if(!max_error)
                        match_filter82_in[j]=signala[j]*signalc[dex];
                    else
                        match_filter82_in[j]=signalb[j]*signalc[dex];
                }
                fft_execute(m82x);
                
                volk_32fc_magnitude_squared_32f(p_value, match_filter82_out, Na);
                for(uint32_t k=0;k<Na;k++){
                    value_sqrt=std::sqrt(p_value[k]);
                    p_sqrt[k]=value_sqrt;
                    if(value_sqrt>max){
                        max=value_sqrt;
                        value[i]=k;
                    }
                }
                //std::cout<<"match_value["<<i<<"]: "<<(float)value[i]<<std::endl;
                if(max_error){
					if(i!=0){
						if((value[i]==value[i-1])){
							//std::cout<<"error value["<<i<<"/"<<i-1<<"]: "<<value[i]<<value[i-1]<<std::endl;
								value[i]=1023;
								value[i-1]=1023;
						}
					} 
						max_error=false;
                        max=0;
                        continue;
                                       
                }
                else{
                    max_old=value[i];
                }
                max_temp=max-max/2.5;
                max_error=false;
                max_start=false;
                max_count=0;
                count=0;
                for(uint32_t k=0;k<Na;k++){
                    if( p_sqrt[k]>max_temp){
                        max_temp=p_sqrt[k];
                        if(!max_start)
                            max_start=true;
                    }
                    else{
                        if(max_start){
                            count=count+1;
                            if(count>10){
                                max_count=max_count+1;
                                count=0;
                                max_start=false;
                                max_temp=max-max/2.5;
                            }
                            if(max_count>2){
                                max_error-=true;
                                max_count=0;
                                
                                //std::cout<<"error,value["<<i<<"]: "<<value[i]<<std::endl;
                                //sleep(60);
                                break;
                            }
                        }
                    }
                }
                if(max_error){
                    i=i-1;
                }
                max=0;
            }
            //return result_value;
        }
        
        float decoder_impl::xcorr(const gr_complex * signala, const gr_complex * signalb, gr_complex * result=NULL,  uint32_t Na=2,uint32_t Nb=2,bool flag=false,uint32_t *dex_i=NULL)
        {
            /*if(result==NULL){
                result = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            }
            gr_complex * signala_ext = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * signalb_ext = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * out_shifted = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * outa = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * outb = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * out = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            gr_complex * outb_conj = (gr_complex *) malloc(sizeof(gr_complex) * (Na+Nb-1));
            
            gettimeofday(&dwStart,NULL);
            fftplan pa = fft_create_plan(Na+Nb-1, &signala_ext[0], &outa[0], LIQUID_FFT_FORWARD, 0);
            fftplan pb = fft_create_plan(Na+Nb-1, &signalb_ext[0], &outb[0], LIQUID_FFT_FORWARD, 0);
            fftplan px = fft_create_plan(Na+Nb-1, &out[0], &result[0], LIQUID_FFT_BACKWARD, 0);
            gettimeofday(&dwEnd,NULL);  
            dwTime = 1000000*(dwEnd.tv_sec-dwStart.tv_sec)+(dwEnd.tv_usec-dwStart.tv_usec);  
            printf("lxP_T: %ld\n",dwTime); */
            
            float result_value=0;
            if((Na==numbles_for_SF)&&(Nb==numbles_for_SF)){
                result_value=xcorrd(signala, signalb, Na,Nb);
            }
            if((Na==upchirp72_len)&&(Nb==numbles_for_SF)){
                result_value=xcorr72x(signala, signalb, Na,Nb,flag);
            }
            if((Na==upchirp82_len)&&(Nb==numbles_for_SF)){
                result_value=xcorr82x(signala, signalb, Na,Nb,flag,dex_i);
            }
            if((Na==upchirp92_len)&&(Nb==numbles_for_SF)){
                result_value=xcorr92x(signala, signalb, Na,Nb);
            }
            if((Na==upchirp102_len)&&(Nb==numbles_for_SF)){
                result_value=xcorr102x(signala, signalb, Na,Nb);
            }
            //zeropadding

            return result_value;
        }
        
        float decoder_impl::xcorr72x(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb,bool flag=false){
            memset (signala_ext72x, 0, sizeof(gr_complex) * (Nb - 1));
            memcpy (signala_ext72x + (Nb - 1), signala, sizeof(gr_complex) * Na);
            memcpy (signalb_ext72x, signalb, sizeof(gr_complex) * Nb);
            memset (signalb_ext72x + Nb, 0, sizeof(gr_complex) * (Na - 1));
            //gettimeofday(&dwStart,NULL); 
            fft_execute(pa72x);
            fft_execute(pb72x);
            /*gettimeofday(&dwEnd,NULL);  
            dwTime = 1000000*(dwEnd.tv_sec-dwStart.tv_sec)+(dwEnd.tv_usec-dwStart.tv_usec);  
            printf("                       P_T: %ld\n",dwTime); */
            volk_32fc_conjugate_32fc(outb_conj72x, outb72x, Na+Nb-1);
            gr_complex scale = 1.0/(Na+Nb-1);
            for (uint32_t i = 0; i < Na+Nb-1; i++)
                out72x[i] = outa72x[i] * outb_conj72x[i] * scale;

            fft_execute(px72x);
            if(flag==true){
                //samples_to_file("/home/lx/decode_lora/data_corr71.bin", &result72x[0], Na+Nb-1, sizeof(gr_complex));
            }
            
            //fftw_cleanup();
            float * resule_float = (float *) malloc(sizeof(float) * (Na+Nb-1));
            memset (resule_float, 0, sizeof(float) * (Na+Nb-1));
            volk_32fc_magnitude_squared_32f(resule_float,result72x,Na+Nb-1);
            float max_corr=*std::max_element(resule_float,resule_float+Na+Nb-1);
            
            return std::sqrt(max_corr);
        }
        float decoder_impl::xcorr82x(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb,bool flag=false,uint32_t *dex_i=NULL){
             
            memcpy (signala_ext82x, signala, sizeof(gr_complex) * Na);
            memset (signala_ext82x + Na, 0, sizeof(gr_complex) * (Nb-1));
            memset (signalb_ext82x, 0, sizeof(gr_complex) * (Na-1));
            memcpy (signalb_ext82x + (Na-1), signalb, sizeof(gr_complex) * Nb);
            //std::cout<<"hello"<<std::endl;
            fft_execute(pa82x);
            fft_execute(pb82x);
            
            volk_32fc_conjugate_32fc(outa_conj82x, outa82x, Na+Nb-1);
            gr_complex scale = 1.0/(Na+Nb-1);
           // gr_complex cmxlx   = gr_complex(2.0f, 2.0f);
            for (uint32_t i = 0; i < Na+Nb-1; i++)
                out82x[i] = outb82x[i] * outa_conj82x[i] * scale;
            
            fft_execute(px82x);
            
            if(flag==true){
                //samples_to_file("/home/lx/decode_lora/data_corr71.bin", &result82x[0], Na+Nb-1, sizeof(gr_complex));
            }
            //fftw_cleanup();
            float * resule_float = (float *) malloc(sizeof(float) * (Na+Nb-1));
            memset (resule_float, 0, sizeof(float) * (Na+Nb-1));
            volk_32fc_magnitude_squared_32f(resule_float,result82x,Na+Nb-1);
            float max_corr=*std::max_element(resule_float,resule_float+Na+Nb-1);
            float limit_max=max_corr/4;
            bool limit_flag=false;
            uint32_t limit_count=0;
            for(uint32_t i=0;i<Na+Nb-1;i++)
            {
                if(*(resule_float+i)>limit_max){
                    *dex_i=i+1;
                    limit_max=*(resule_float+i);
                    limit_flag=true;
                }
                else{
                    if(limit_flag){
                        limit_count=limit_count+1;
                        if(limit_count>30){
                            break;
                        }
                    }
                }
            }
            
            return std::sqrt(max_corr);
        }
        float decoder_impl::xcorr92x(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb){
            memset (signala_ext92x, 0, sizeof(gr_complex) * (Nb - 1));
            memcpy (signala_ext92x + (Nb - 1), signala, sizeof(gr_complex) * Na);
            memcpy (signalb_ext92x, signalb, sizeof(gr_complex) * Nb);
            memset (signalb_ext92x + Nb, 0, sizeof(gr_complex) * (Na - 1));
            
            fft_execute(pa92x);
            fft_execute(pb92x);
            
            volk_32fc_conjugate_32fc(outb_conj92x, outb92x, Na+Nb-1);
            gr_complex scale = 1.0/(Na+Nb-1);
            for (uint32_t i = 0; i < Na+Nb-1; i++)
                out92x[i] = outa92x[i] * outb_conj92x[i] * scale;

            fft_execute(px92x);
            
            //fftw_cleanup();
            float * resule_float = (float *) malloc(sizeof(float) * (Na+Nb-1));
            memset (resule_float, 0, sizeof(float) * (Na+Nb-1));
            volk_32fc_magnitude_squared_32f(resule_float,result92x,Na+Nb-1);
            float max_corr=*std::max_element(resule_float,resule_float+Na+Nb-1);
            
            return std::sqrt(max_corr);
        }
        float decoder_impl::xcorr102x(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb){
            memset (signala_ext102x, 0, sizeof(gr_complex) * (Nb - 1));
            memcpy (signala_ext102x + (Nb - 1), signala, sizeof(gr_complex) * Na);
            memcpy (signalb_ext102x, signalb, sizeof(gr_complex) * Nb);
            memset (signalb_ext102x + Nb, 0, sizeof(gr_complex) * (Na - 1));
            
            fft_execute(pa102x);
            fft_execute(pb102x);
            
            volk_32fc_conjugate_32fc(outb_conj102x, outb102x, Na+Nb-1);
            gr_complex scale = 1.0/(Na+Nb-1);
            for (uint32_t i = 0; i < Na+Nb-1; i++)
                out102x[i] = outa102x[i] * outb_conj102x[i] * scale;

            fft_execute(px102x);

            
            //fftw_cleanup();
            float * resule_float = (float *) malloc(sizeof(float) * (Na+Nb-1));
            memset (resule_float, 0, sizeof(float) * (Na+Nb-1));
            volk_32fc_magnitude_squared_32f(resule_float,result102x,Na+Nb-1);
            float max_corr=*std::max_element(resule_float,resule_float+Na+Nb-1);
            
            return std::sqrt(max_corr);
        }
        float decoder_impl::xcorrd(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb){
            memset (signala_extd, 0, sizeof(gr_complex) * (Nb - 1));
            memcpy (signala_extd + (Nb - 1), signala, sizeof(gr_complex) * Na);
            memcpy (signalb_extd, signalb, sizeof(gr_complex) * Nb);
            memset (signalb_extd + Nb, 0, sizeof(gr_complex) * (Na - 1));
            
            fft_execute(pad);
            fft_execute(pbd);
            
            volk_32fc_conjugate_32fc(outb_conjd, outbd, Na+Nb-1);
            gr_complex scale = 1.0/(Na+Nb-1);
            for (uint32_t i = 0; i < Na+Nb-1; i++)
                outd[i] = outad[i] * outb_conjd[i] * scale;

            fft_execute(pxd);

            
            //fftw_cleanup();
            float * resule_float = (float *) malloc(sizeof(float) * (Na+Nb-1));
            memset (resule_float, 0, sizeof(float) * (Na+Nb-1));
            volk_32fc_magnitude_squared_32f(resule_float,resultd,Na+Nb-1);
            float max_corr=*std::max_element(resule_float,resule_float+Na+Nb-1);
            
            return std::sqrt(max_corr);
        }
        
        float decoder_impl::xcorr_mini_time_interval(const gr_complex * signala, const gr_complex * signalb,uint32_t Na,uint32_t Nb){
            memcpy (signala_time_intervalx, signala, sizeof(gr_complex) * Na);
            memset (signala_time_intervalx + Na, 0, sizeof(gr_complex) * (Nb-1));
            memset (signalb_time_intervalx, 0, sizeof(gr_complex) * (Na-1));
            memcpy (signalb_time_intervalx + (Na-1), signalb, sizeof(gr_complex) * Nb);
            //std::cout<<"hello"<<std::endl;
            fft_execute(pa_time_intervalx);
            fft_execute(pb_time_intervalx);
            
            volk_32fc_conjugate_32fc(outa_conj_timex, outa_time_intervalx, Na+Nb-1);
            gr_complex scale = 1.0/(Na+Nb-1);
           // gr_complex cmxlx   = gr_complex(2.0f, 2.0f);
            for (uint32_t i = 0; i < Na+Nb-1; i++)
                out_time_intervalx[i] = outb_time_intervalx[i] * outa_conj_timex[i] * scale;
            
            fft_execute(px_time_intervalx);
            
            //fftw_cleanup();
            float * resule_float = (float *) malloc(sizeof(float) * (Na+Nb-1));
            memset (resule_float, 0, sizeof(float) * (Na+Nb-1));
            volk_32fc_magnitude_squared_32f(resule_float,result_timex,Na+Nb-1);
            float max_corr=*std::max_element(resule_float,resule_float+Na+Nb-1);
            float limit_max=max_corr/4;
            bool limit_flag=false;
            uint32_t limit_count=0;
            uint32_t dex_i=0;
            for(uint32_t i=0;i<Na+Nb-1;i++)
            {
                if(*(resule_float+i)>limit_max){
                    dex_i=i+1;
                    limit_max=*(resule_float+i);
                    limit_flag=true;
                    limit_count=0;
                }
                else{
                    if(limit_flag){
                        limit_count=limit_count+1;
                        if(limit_count>200){
                            break;
                        }
                    }
                }
            }
            
            return (float)dex_i;
        }


        void decoder_impl::build_ideal_chirps(void) {
            d_downchirp.resize(d_samples_per_symbol);
            d_upchirp.resize(d_samples_per_symbol);
            d_downchirp_ifreq.resize(d_samples_per_symbol);
            d_upchirp_ifreq.resize(d_samples_per_symbol);
            d_upchirp_ifreq_v.resize(d_samples_per_symbol*3);
            gr_complex tmp[d_samples_per_symbol*3];
            upchirp_3=(gr_complex *) malloc(sizeof(gr_complex) * (d_samples_per_symbol*3));

            const double T       = -0.5 * d_bw * d_symbols_per_second;
            const double f0      = (d_bw / 2.0);
            const double pre_dir = 2.0 * M_PI;
            double t;
            gr_complex cmx       = gr_complex(1.0f, 1.0f);

            for (uint32_t i = 0u; i < d_samples_per_symbol; i++) {
                // Width in number of samples = samples_per_symbol
                // See https://en.wikipedia.org/wiki/Chirp#Linear
                t = d_dt * i;//+d_dt/4.0; //for example delay
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
            memcpy(upchirp_3, tmp, sizeof(gr_complex) * (d_samples_per_symbol*3));
            instantaneous_frequency(tmp, &d_upchirp_ifreq_v[0], d_samples_per_symbol*3);
        }
        
        void decoder_impl::build_ideal_chirps_lx(float samp_rate,uint32_t bandwidth, uint8_t sf) {         //lx
            double dt_lx=1.0f / samp_rate;
            double symbols_per_second_lx=(double)bandwidth / (1u << sf);
            uint32_t samples_per_symbol_lx=(uint32_t)(samp_rate / symbols_per_second_lx);
            d_upchirplx.resize(samples_per_symbol_lx);
            d_upchirplx_len=samples_per_symbol_lx;

            const double T       = -0.5 * bandwidth * symbols_per_second_lx;
            const double f0      = (bandwidth / 2.0);
            const double pre_dir = 2.0 * M_PI;
            double t;
            gr_complex cmx       = gr_complex(10.0f, 10.0f);
            uint32_t j = 0u;
            for (uint32_t i = 0u; i < samples_per_symbol_lx;j++ ) {                
                t = dt_lx * i;
                d_upchirplx[j]   = cmx * gr_expj(pre_dir * t * (f0 + T * t) * -1.0f);
                i=i+1;
            }
        }
        
        uint32_t decoder_impl::ideal_chirps_num(float samp_rate,uint32_t bandwidth, uint8_t sf) {         //lx
            double symbols_per_second_lx=(double)bandwidth / (1u << sf);
            uint32_t samples_per_symbol_lx=(uint32_t)(samp_rate / symbols_per_second_lx);
            return samples_per_symbol_lx;
        }
        
        void decoder_impl::build_ideal_chirps_sf_bw(float samp_rate,uint32_t bandwidth, uint8_t sf,gr_complex* d_upchirplx,gr_complex* d_dnchirplx) {         //lx
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
                d_dnchirplx[i] = cmx * gr_expj(pre_dir * t * (f0 + T * t));
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
        void decoder_impl::float_values_to_file(const std::string path, float *v, uint32_t length, uint32_t elem_size) {
             std::ofstream out_file;
                out_file.open(path.c_str(), std::ios::out | std::ios::binary );

                for (uint32_t i = 0u; i < length; i++) {
                    out_file.write(reinterpret_cast<const char *>(&v[i]), elem_size);
                }

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
                //sleep(60);
            #else
                (void) path;
                (void) v;
                (void) length;
                (void) elem_size;
            #endif
        }
        void decoder_impl::time_to_file_add(const std::string path, double time_value ){
            std::ofstream out_file;
            out_file.open(path.c_str(), std::ios::out | std::ios::app);
            char * pCh= new char[30];
            uint8_t value_len=0;
            std::cout << boost::format("time_value:  %.9f seconds") % (time_value)<<std::endl;
            sprintf(pCh,"%.9lf%c",time_value,'A');
            for(int i=0;i<30;i++)
            {
                if(*(pCh+i)=='A'){
                    value_len=i;
                    break;
                }
            }
            
            std::cout<<"pCh: "<<pCh<<std::endl;
            out_file.write(&pCh[0],value_len);
            out_file.write("\n",1);
            out_file.write("\n",1);
            out_file.close();
            delete pCh;
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

        void decoder_impl::fine_synclx(const gr_complex* in_samples, uint32_t bin_idx, int32_t search_space) {
            int32_t shift_ref = (bin_idx+1) * d_decim_factor;
            float samples_ifreq[d_samples_per_symbol];
            float max_correlation = 0.0f;
            int32_t lag = 0;

            instantaneous_frequency(in_samples, samples_ifreq, d_samples_per_symbol);

            for(int32_t i = -search_space+1; i < search_space; i++) {
                float c = cross_correlate_fast(in_samples, &upchirp_3[shift_ref+i+d_samples_per_symbol], d_samples_per_symbol);
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
             float dete_value[numbers_para]={0};
            
             float corr_value71=xcorr(&d_upchirp71[0], &input[0], NULL, upchirp71_len,numbles_for_SF);
             dete_value[0]=10*corr_value71/std::sqrt(auto_corr_D*auto_corr71);
            
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
        
        bool decoder_impl::demodulatelx(const gr_complex *samples, const bool reduced_rate) {
            // DBGR_TIME_MEASUREMENT_TO_FILE("SFxx_method");

            // DBGR_START_TIME_MEASUREMENT(false, "only");

            uint32_t count=7;
            uint32_t bin_idx=0;
            float* value=(float *) malloc(sizeof(float) * (count));
            match_filter(&d_dnchirp71[0], &d_upchirp71[0], &samples[0],value, count,upchirp71_len);
           
            if(value[0]!=0){
                bin_idx = (uint32_t)value[0]-1;
            }
            else{
                bin_idx = 0;
            }
            
            if(bin_idx>800)
                bin_idx=bin_idx-896;
            //uint32_t bin_idx = get_shift_fft(samples);
            //std::cout<<"bin_idx: "<< (float)bin_idx<<std::endl;
            if(d_enable_fine_sync){
                fine_synclx(samples, bin_idx, std::max(d_decim_factor / 4u, 2u));
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


        bool decoder_impl::demodulate(const gr_complex *samples, const bool reduced_rate) {
            // DBGR_TIME_MEASUREMENT_TO_FILE("SFxx_method");

            // DBGR_START_TIME_MEASUREMENT(false, "only");

            uint32_t bin_idx = max_frequency_gradient_idx(samples);
            std::cout<<"bin_idx1: "<<(float)bin_idx<<std::endl;
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
            
            static int detect_count=0;
            //static uint32_t BW_LX=250000;
            //static int SF=0;
            //static int lxcount=1;
            //float corr_value1=0;
            //float corr_value2=0;
            //float corr_value3=0;
            uint8_t sf_BW_temp[2]={0,0};
            
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
                            build_ideal_chirps_lx(sample_lx,125000, SF-1);
                            corr_value1=correlate_detSFBW_lx(&out_sample_d4[0], corr_len);
                            build_ideal_chirps_lx(sample_lx,250000, SF);
                            corr_value2=correlate_detSFBW_lx(&out_sample_d4[0], corr_len);
                            build_ideal_chirps_lx(sample_lx,500000, SF+1);
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
            //gettimeofday(&dwStart,NULL); 
            //std::cout<<"start "<<std::endl;
            //static bool dete_noise=true;
            int num_end_pro=0;
            static bool start_first=true;
            bool x71_flag,x72_flag,x81_flag,x82_flag,x85_flag,x91_flag,x92_flag,x95_flag,x102_flag,x105_flag;
            x71_flag=false;
            //x72_flag=false;
            /*x81_flag=false;
            x82_flag=false;
            x85_flag=false;
            x91_flag=false;
            x92_flag=false;
            x95_flag=false;
            x102_flag=false;
            x105_flag=false;*/
            
            
            char buf[256]={' '};
            char buf1[15]={' '};
            char buf2[10]={' '};
            uint32_t start_dex71=0;
            int returned_count=0;
            static int lx_flag=0;
            static int hahacount=0;
            
            
            if(start_first){
                signal(2,handler);
                p_map=(gr_complex*)mmap(NULL,sizeof(gr_complex)*numbles_for_SF,PROT_READ|PROT_WRITE,MAP_SHARED|MAP_ANONYMOUS,-1,0);
                
                start_first=false;
                child71= fork();
                if(child71 == 0)
                {
                     //sleep(1);
                    //std::cout<<"child71: "<<child71<<std::endl;
                    double auto_corr_D=0;
                    while(1){
                        close(file_fathertox711[OUTPUT]);
                        //printf("start read71: %d\n",returned_count);
                        memset (buf, ' ', sizeof(char) * (255));
                        while(returned_count==0){
                            returned_count = read(file_fathertox711[INPUT], buf, sizeof(buf));
                        }
                        //printf("%d bytes of data received from spawned 71process: %s\n",returned_count, buf);
                        returned_count=0;
                        memset (buf, ' ', sizeof(char) * (255));
                        
                        uint32_t dex_71=0;
                        float corr_value711=xcorr(&d_upchirp71[0], &p_map[0], NULL, upchirp71_len,numbles_for_SF,true,&dex_71);
                        
                        //std::cout<<"dex_71: "<<(float)dex_71<<std::endl;
                        
                        memset (buf, ' ', sizeof(char) * (255));
                        close(file_fathertox71[OUTPUT]);
                        //printf("wait d_auto71: %d\n",returned_count);
                        while(returned_count==0){
                            returned_count = read(file_fathertox71[INPUT], buf, sizeof(buf));
                        }
                        auto_corr_D=atof(buf)/10000.0;
                        //printf("%d bytes of data received from spawned 71process: %s\n",returned_count, buf);
                        returned_count=0;

                        //std::cout<<"auto_corr71: "<<auto_corr71<<std::endl;
                        float dete_value=10*corr_value711/std::sqrt(auto_corr_D*auto_corr71); 
                        //std::cout<<"dete_value: "<<dete_value<<std::endl;
                        
                        memset (buf, ' ', sizeof(char) * (255));
                        sprintf(buf,"%.6f",dete_value);
                        sprintf(buf,"%s%c",buf,'+');
                        sprintf(buf,"%s%d",buf,dex_71);
                        sprintf(buf,"%s%c",buf,'#');
                        int i=0;
                        for(i;i<255;i++){
                            if(buf[i]==' '){
                                break;
                            }
                        }
                        //std::cout<<"i: "<<i<<std::endl;
                        //printf("x71tofather has send\n");
                        close(file_x71tofather[INPUT]);
                        write(file_x71tofather[OUTPUT], buf, i); 
                        
                    }
                    exit(0);
                }
                /*else{
                    child72= fork();
                    if(child72 == 0)
                    {
                        double auto_corr_D=0;
                        while(1){
                            //******wait start****
                            close(file_fathertox721[OUTPUT]);
                            memset (buf, ' ', sizeof(char) * (255));
                            while(returned_count==0){
                                returned_count = read(file_fathertox721[INPUT], buf, sizeof(buf));
                            }
                            //******calculate xcorr****
                            returned_count=0;
                            memset (buf, ' ', sizeof(char) * (255));
                            float corr_value72=xcorr(&d_upchirp72[0], &p_map[0], NULL, upchirp72_len,numbles_for_SF,true);
                            //******wait data_auto_corr for normalized****
                            memset (buf, ' ', sizeof(char) * (255));
                            close(file_fathertox72[OUTPUT]);
                            while(returned_count==0){
                                returned_count = read(file_fathertox72[INPUT], buf, sizeof(buf));
                            }
                            auto_corr_D=atof(buf)/10000.0;
                            returned_count=0;
                            float dete_value=10*corr_value72/std::sqrt(auto_corr_D*auto_corr72); 
                            //******send normalized dete_value to father process****
                            memset (buf, ' ', sizeof(char) * (255));
                            sprintf(buf,"%.6f",dete_value);
                            int i=0;
                            for(i;i<255;i++){
                                if(buf[i]==' '){
                                    break;
                                }
                            }
                            close(file_x72tofather[INPUT]);
                            write(file_x72tofather[OUTPUT], buf, i); 
                        }
                        exit(0);
                    }
                    else{
                        child81= fork();
                        if(child81== 0)
                        {
                            double auto_corr_D=0;
                            while(1){
                                //******wait start****
                                close(file_fathertox811[OUTPUT]);
                                memset (buf, ' ', sizeof(char) * (255));
                                while(returned_count==0){
                                    returned_count = read(file_fathertox811[INPUT], buf, sizeof(buf));
                                }
                                //******calculate xcorr****
                                returned_count=0;
                                memset (buf, ' ', sizeof(char) * (255));
                                float corr_value81=xcorr(&d_upchirp81[0], &p_map[0], NULL, upchirp81_len,numbles_for_SF);
                                //******wait data_auto_corr for normalized****
                                memset (buf, ' ', sizeof(char) * (255));
                                close(file_fathertox81[OUTPUT]);
                                while(returned_count==0){
                                    returned_count = read(file_fathertox81[INPUT], buf, sizeof(buf));
                                }
                                auto_corr_D=atof(buf)/10000.0;
                                returned_count=0;
                                float dete_value=10*corr_value81/std::sqrt(auto_corr_D*auto_corr81); 
                                //******send normalized dete_value to father process****
                                memset (buf, ' ', sizeof(char) * (255));
                                sprintf(buf,"%.6f",dete_value);
                                int i=0;
                                for(i;i<255;i++){
                                    if(buf[i]==' '){
                                        break;
                                    }
                                }
                                close(file_x81tofather[INPUT]);
                                write(file_x81tofather[OUTPUT], buf, i); 
                            }
                            exit(0);
                        }
                       else{
                            child82= fork();
                            if(child82== 0)
                            {
                                double auto_corr_D=0;
                                while(1){
                                    //******wait start****
                                    close(file_fathertox821[OUTPUT]);
                                    memset (buf, ' ', sizeof(char) * (255));
                                    while(returned_count==0){
                                        returned_count = read(file_fathertox821[INPUT], buf, sizeof(buf));
                                    }
                                    //******calculate xcorr****
                                    returned_count=0;
                                    memset (buf, ' ', sizeof(char) * (255));
                                    float corr_value82=xcorr(&d_upchirp82[0], &p_map[0], NULL, upchirp82_len,numbles_for_SF);
                                    //******wait data_auto_corr for normalized****
                                    memset (buf, ' ', sizeof(char) * (255));
                                    close(file_fathertox82[OUTPUT]);
                                    while(returned_count==0){
                                        returned_count = read(file_fathertox82[INPUT], buf, sizeof(buf));
                                    }
                                    auto_corr_D=atof(buf)/10000.0;
                                    returned_count=0;
                                    float dete_value=10*corr_value82/std::sqrt(auto_corr_D*auto_corr82); 
                                    //******send normalized dete_value to father process****
                                    memset (buf, ' ', sizeof(char) * (255));
                                    sprintf(buf,"%.6f",dete_value);
                                    int i=0;
                                    for(i;i<255;i++){
                                        if(buf[i]==' '){
                                            break;
                                        }
                                    }
                                    close(file_x82tofather[INPUT]);
                                    write(file_x82tofather[OUTPUT], buf, i); 
                                }
                                exit(0);
                            }
                            else{
                                child85= fork();
                                if(child85== 0)
                                {
                                    double auto_corr_D=0;
                                    while(1){
                                        //******wait start****
                                        close(file_fathertox851[OUTPUT]);
                                        memset (buf, ' ', sizeof(char) * (255));
                                        while(returned_count==0){
                                            returned_count = read(file_fathertox851[INPUT], buf, sizeof(buf));
                                        }
                                        //******calculate xcorr****
                                        returned_count=0;
                                        memset (buf, ' ', sizeof(char) * (255));
                                        float corr_value85=xcorr(&d_upchirp85[0], &p_map[0], NULL, upchirp85_len,numbles_for_SF);
                                        //******wait data_auto_corr for normalized****
                                        memset (buf, ' ', sizeof(char) * (255));
                                        close(file_fathertox85[OUTPUT]);
                                        while(returned_count==0){
                                            returned_count = read(file_fathertox85[INPUT], buf, sizeof(buf));
                                        }
                                        auto_corr_D=atof(buf)/10000.0;
                                        returned_count=0;
                                        float dete_value=10*corr_value85/std::sqrt(auto_corr_D*auto_corr85); 
                                        //******send normalized dete_value to father process****
                                        memset (buf, ' ', sizeof(char) * (255));
                                        sprintf(buf,"%.6f",dete_value);
                                        int i=0;
                                        for(i;i<255;i++){
                                            if(buf[i]==' '){
                                                break;
                                            }
                                        }
                                        close(file_x85tofather[INPUT]);
                                        write(file_x85tofather[OUTPUT], buf, i); 
                                    }
                                    exit(0);
                                }
                                else{
                                    child91= fork();
                                    if(child91== 0)
                                    {
                                        double auto_corr_D=0;
                                        while(1){
                                            //******wait start****
                                            close(file_fathertox911[OUTPUT]);
                                            memset (buf, ' ', sizeof(char) * (255));
                                            while(returned_count==0){
                                                returned_count = read(file_fathertox911[INPUT], buf, sizeof(buf));
                                            }
                                            //******calculate xcorr****
                                            returned_count=0;
                                            memset (buf, ' ', sizeof(char) * (255));
                                            //gettimeofday(&dwStart,NULL); 
                                            float corr_value91=xcorr(&d_upchirp91[0], &p_map[0], NULL, upchirp81_len,numbles_for_SF);
                                            //gettimeofday(&dwEnd,NULL);  
                                            //dwTime = 1000000*(dwEnd.tv_sec-dwStart.tv_sec)+(dwEnd.tv_usec-dwStart.tv_usec);  
                                            //printf("              P_T11: %ld\n",dwTime); 
                                            //******wait data_auto_corr for normalized****
                                            memset (buf, ' ', sizeof(char) * (255));
                                            close(file_fathertox91[OUTPUT]);
                                            while(returned_count==0){
                                                returned_count = read(file_fathertox91[INPUT], buf, sizeof(buf));
                                            }
                                            auto_corr_D=atof(buf)/10000.0;
                                            returned_count=0;
                                            float dete_value=10*corr_value91/std::sqrt(auto_corr_D*auto_corr91); 
                                            //******send normalized dete_value to father process****
                                            memset (buf, ' ', sizeof(char) * (255));
                                            sprintf(buf,"%.6f",dete_value);
                                            int i=0;
                                            for(i;i<255;i++){
                                                if(buf[i]==' '){
                                                    break;
                                                }
                                            }
                                            close(file_x91tofather[INPUT]);
                                            write(file_x91tofather[OUTPUT], buf, i); 
                                        }
                                        exit(0);
                                    }*/
                                    /*else{
                                        child92= fork();
                                        if(child92== 0)
                                        {
                                            double auto_corr_D=0;
                                            while(1){
                                                //******wait start****
                                                close(file_fathertox921[OUTPUT]);
                                                memset (buf, ' ', sizeof(char) * (255));
                                                while(returned_count==0){
                                                    returned_count = read(file_fathertox921[INPUT], buf, sizeof(buf));
                                                }
                                                //******calculate xcorr****
                                                returned_count=0;
                                                memset (buf, ' ', sizeof(char) * (255));
                                                float corr_value92=xcorr(&d_upchirp92[0], &p_map[0], NULL, upchirp92_len,numbles_for_SF);
                                                //******wait data_auto_corr for normalized****
                                                memset (buf, ' ', sizeof(char) * (255));
                                                close(file_fathertox92[OUTPUT]);
                                                while(returned_count==0){
                                                    returned_count = read(file_fathertox92[INPUT], buf, sizeof(buf));
                                                }
                                                auto_corr_D=atof(buf)/10000.0;
                                                returned_count=0;
                                                float dete_value=10*corr_value92/std::sqrt(auto_corr_D*auto_corr92); 
                                                //******send normalized dete_value to father process****
                                                memset (buf, ' ', sizeof(char) * (255));
                                                sprintf(buf,"%.6f",dete_value);
                                                int i=0;
                                                for(i;i<255;i++){
                                                    if(buf[i]==' '){
                                                        break;
                                                    }
                                                }
                                                close(file_x92tofather[INPUT]);
                                                write(file_x92tofather[OUTPUT], buf, i); 
                                            }
                                            exit(0);
                                        }
                                        else{
                                            child95= fork();
                                            if(child95== 0)
                                            {
                                                double auto_corr_D=0;
                                                while(1){
                                                    //******wait start****
                                                    close(file_fathertox951[OUTPUT]);
                                                    memset (buf, ' ', sizeof(char) * (255));
                                                    while(returned_count==0){
                                                        returned_count = read(file_fathertox951[INPUT], buf, sizeof(buf));
                                                    }
                                                    //******calculate xcorr****
                                                    returned_count=0;
                                                    memset (buf, ' ', sizeof(char) * (255));
                                                    float corr_value95=xcorr(&d_upchirp95[0], &p_map[0], NULL, upchirp95_len,numbles_for_SF);
                                                    //******wait data_auto_corr for normalized****
                                                    memset (buf, ' ', sizeof(char) * (255));
                                                    close(file_fathertox95[OUTPUT]);
                                                    while(returned_count==0){
                                                        returned_count = read(file_fathertox95[INPUT], buf, sizeof(buf));
                                                    }
                                                    auto_corr_D=atof(buf)/10000.0;
                                                    returned_count=0;
                                                    float dete_value=10*corr_value95/std::sqrt(auto_corr_D*auto_corr95); 
                                                    //******send normalized dete_value to father process****
                                                    memset (buf, ' ', sizeof(char) * (255));
                                                    sprintf(buf,"%.6f",dete_value);
                                                    int i=0;
                                                    for(i;i<255;i++){
                                                        if(buf[i]==' '){
                                                            break;
                                                        }
                                                    }
                                                    close(file_x95tofather[INPUT]);
                                                    write(file_x95tofather[OUTPUT], buf, i); 
                                                }
                                                exit(0);
                                            }
                                            else{
                                                child102= fork();
                                                if(child102== 0)
                                                {
                                                    double auto_corr_D=0;
                                                    while(1){
                                                        //******wait start****
                                                        close(file_fathertox1021[OUTPUT]);
                                                        memset (buf, ' ', sizeof(char) * (255));
                                                        while(returned_count==0){
                                                            returned_count = read(file_fathertox1021[INPUT], buf, sizeof(buf));
                                                        }
                                                        //******calculate xcorr****
                                                        returned_count=0;
                                                        memset (buf, ' ', sizeof(char) * (255));
                                                        float corr_value102=xcorr(&d_upchirp102[0], &p_map[0], NULL, upchirp92_len,numbles_for_SF);
                                                        //******wait data_auto_corr for normalized****
                                                        memset (buf, ' ', sizeof(char) * (255));
                                                        close(file_fathertox102[OUTPUT]);
                                                        while(returned_count==0){
                                                            returned_count = read(file_fathertox102[INPUT], buf, sizeof(buf));
                                                        }
                                                        auto_corr_D=atof(buf)/10000.0;
                                                        returned_count=0;
                                                        float dete_value=10*corr_value102/std::sqrt(auto_corr_D*auto_corr102); 
                                                        //******send normalized dete_value to father process****
                                                        memset (buf, ' ', sizeof(char) * (255));
                                                        sprintf(buf,"%.6f",dete_value);
                                                        int i=0;
                                                        for(i;i<255;i++){
                                                            if(buf[i]==' '){
                                                                break;
                                                            }
                                                        }
                                                        close(file_x102tofather[INPUT]);
                                                        write(file_x102tofather[OUTPUT], buf, i); 
                                                    }
                                                    exit(0);
                                                }
                                                else{
                                                    child105= fork();
                                                    if(child105== 0)
                                                    {
                                                        double auto_corr_D=0;
                                                        while(1){
                                                            //******wait start****
                                                            close(file_fathertox1051[OUTPUT]);
                                                            memset (buf, ' ', sizeof(char) * (255));
                                                            while(returned_count==0){
                                                                returned_count = read(file_fathertox1051[INPUT], buf, sizeof(buf));
                                                            }
                                                            //******calculate xcorr****
                                                            returned_count=0;
                                                            memset (buf, ' ', sizeof(char) * (255));
                                                            float corr_value105=xcorr(&d_upchirp105[0], &p_map[0], NULL, upchirp105_len,numbles_for_SF);
                                                            //******wait data_auto_corr for normalized****
                                                            memset (buf, ' ', sizeof(char) * (255));
                                                            close(file_fathertox105[OUTPUT]);
                                                            while(returned_count==0){
                                                                returned_count = read(file_fathertox105[INPUT], buf, sizeof(buf));
                                                            }
                                                            auto_corr_D=atof(buf)/10000.0;
                                                            returned_count=0;
                                                            float dete_value=10*corr_value105/std::sqrt(auto_corr_D*auto_corr105); 
                                                            //******send normalized dete_value to father process****
                                                            memset (buf, ' ', sizeof(char) * (255));
                                                            sprintf(buf,"%.6f",dete_value);
                                                            int i=0;
                                                            for(i;i<255;i++){
                                                                if(buf[i]==' '){
                                                                    break;
                                                                }
                                                            }
                                                            close(file_x105tofather[INPUT]);
                                                            write(file_x105tofather[OUTPUT], buf, i); 
                                                        }
                                                        exit(0);
                                                    }
                                                }
                                            }
                                        }
                                    }*/
                              //  }
                           // }
                       // }
                   // }
               // }
            }
           
            if(lx_flag==1)
            {  
                switch (d_state) {
                    case gr::lora::DecoderState::SYNC: {
                        
                        //double dex_find=find_mini_time_interval(&input[0]);
                        //std::cout<<"Dex: "<<(float)(dex_find)<<std::endl;
						//std::cout << boost::format("before time_uart::  %.9f seconds\n") % (time_uart)<<std::endl;
						//time_uart=time_uart+double(dex_find/1000000.0/16.0);
						//std::cout << boost::format("deta time::  %.9f seconds\n") % (double(dex_find/1000000.0/16.0))<<std::endl;
						//std::cout << boost::format("after time_uart::  %.9f seconds\n") % (time_uart)<<std::endl;
                        
                        uint32_t count=2;
                        float* value=(float *) malloc(sizeof(float) * (count));
                        match_filter(&d_dnchirp71[0], &d_upchirp71[0], &input[0],value, count,upchirp71_len);
                        for(int i=0;i<2;i++){
                            if(value[i]>800)
                                value[i]=value[i]-896;
                        }
                        //std::cout<<"SYNC: value[0/1]"<<value[0]<<"/ "<<value[1]<<std::endl;
                        if(value[0]==0&&value[1]==0){
                            if(hahacount==100){
                                std::cout<<"Have Received 100 messages!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
                                while(1);
                            }
							samples_to_file("/home/lx/decode_lora/sync.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
							//sleep(60);
                            hahacount=hahacount+1;
                            
                            d_state = gr::lora::DecoderState::FIND_SFD;
                            consume_each(upchirp71_len);
                        }
                        else{
                            if(value[0]==value[1]){
                                uint32_t consum_count=0;
                                int dex=0;
                                float value_old=value[0];
                                gr_complex* upchirp_temp=(gr_complex *) malloc(sizeof(gr_complex) * (upchirp71_len));
                                count=1;
                                for(int i=1;i<9;i++){
                                    memcpy(upchirp_temp, &input[0]+i, sizeof(gr_complex) * upchirp71_len);
                                    match_filter(&d_dnchirp71[0], &d_upchirp71[0], upchirp_temp,value, count,upchirp71_len);
                                    if(value[0]>800)
                                        value[0]=value[0]-896;
                                    //std::cout<<"value: "<<value[0]<<std::endl;
                                    if(value[0]==value_old+1){
                                        dex=i;
                                        break;
                                    }
                                }
                                if(dex==0){
                                    if (value_old==0){
                                        dex=6;
                                    }
                                    else{
                                        dex=8;
                                    }
                                }
                                 //std::cout<<"dex: "<<dex<<std::endl;
                                if (value_old==0){
                                    consum_count=1024-6+dex;
                                }
                                else{
                                    consum_count=1024-(value_old+1)*8+dex;
                                }

                                //samples_to_file("/home/lx/decode_lora/sync.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
                                consume_each(consum_count);
                            }
                            else{
                                lx_flag=0;
                                consume_each(upchirp71_len);
                            }
                        }
                        break;
                    }
                        
                    case gr::lora::DecoderState::FIND_SFD: {
                        
                        uint32_t count=(numbles_for_SF-numbles_for_SF%upchirp71_len)/upchirp71_len;
                        float* value=(float *) malloc(sizeof(float) * (count));
                        match_filter(&d_dnchirp71[0], &d_upchirp71[0], &input[0],value, count,upchirp71_len);
                        uint32_t i=0;
                        for(i=0;i<count;i++){
                            //std::cout<<"value2["<<i<<"]: "<<value[i]<<std::endl;
                            
                            if ((value[i]==1023)||(value[i]==113)) {
                                if(value[i+1]==value[i]){//lingshi LXLX
                                    d_state = gr::lora::DecoderState::PAUSE;
                                    consume_each((i+1)*upchirp71_len);
                                    //std::cout<<"1023_dex: "<<i<<std::endl;
                                }
                                else{
                                    lx_flag=0;
                                    d_state = gr::lora::DecoderState::SYNC;
                                    consume_each(numbles_for_SF);
                                }
                                break;
                            } 
                            else {
                                if(value[i]==0) {
                                    //fine_sync(input, d_number_of_bins-1, d_decim_factor * 4);

                                    //std::cout<<"d_fine_sync: "<<d_fine_sync<<std::endl;
                                    //std::cout<<"upchirp"<<std::endl;
                                } else {
                                    d_corr_fails++;
                                    //std::cout<<"noise"<<std::endl;

                                }

                                if (d_corr_fails > 4u) {
                                    lx_flag=0;
                                     d_state = gr::lora::DecoderState::SYNC;
                                    consume_each(numbles_for_SF);
                                    break;
                                }
                            }
                        }
                        if(i==count){
                            consume_each(numbles_for_SF);
                        }
                       
                        break;
                    }

                    case gr::lora::DecoderState::PAUSE: {
                        if(d_implicit){
                            d_state = gr::lora::DecoderState::DECODE_PAYLOAD;
                            d_payload_symbols = 1;
                        } else {
                            d_state = gr::lora::DecoderState::DECODE_HEADER;
                        }
                        consume_each(upchirp71_len + d_delay_after_sync);
                        break;
                    }

                    case gr::lora::DecoderState::DECODE_HEADER: {
                        d_phdr.cr = 4u;
                        //samples_to_file("/home/lx/decode_lora/header.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
                        if (demodulatelx(input, true)) {
                            decode(true);
                            gr::lora::print_vector_hex(std::cout, &d_decoded[0], d_decoded.size(), false);std::cout<<std::endl;
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
                            d_state = gr::lora::DecoderState::DECODE_PAYLOAD;
                        }
                        //std::cout<<"d_fine_sync: "<<(float)d_fine_sync<<std::endl;
                        consume_each(upchirp71_len);//+d_fine_sync
                        
                        break; 
                    }

                    case gr::lora::DecoderState::DECODE_PAYLOAD: {
                        if (d_implicit && determine_energy(input) < d_energy_threshold) {
                            d_payload_symbols = 0;
                            //d_demodulated.erase(d_demodulated.begin(), d_demodulated.begin() + 7u); // Test for SF 8 with header
                            d_payload_length = (int32_t)(d_demodulated.size() / 2);
                        } else if (demodulatelx(input, d_implicit && d_reduced_rate)) {
                            if(!d_implicit)
                                d_payload_symbols -= (4u + d_phdr.cr);
                        }
                        //samples_to_file("/home/lx/decode_lora/payload.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
                       
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
							
							write_uart(time_uart);
                    		time_to_file_add("/home/lx/decode_lora/time.txt",time_uart);
							
                            lx_flag=0;
                            //sleep(60);
                            //message_port_pub(pmt::mp("hahaout"), payload_bloblx);//lx
                            //std::cout<<"out1"<<std::endl;
                            msg_lora_frame();

                            d_state = gr::lora::DecoderState::SYNC;
                            d_decoded.clear();
                            d_words.clear();
                            d_words_dewhitened.clear();
                            d_words_deshuffled.clear();
                            d_demodulated.clear();
                            signal_flag=0;
                        }
                        //std::cout<<"d_fine_sync: "<<(float)d_fine_sync<<std::endl;
                        consume_each(upchirp71_len);//+d_fine_sync

                        break;
                    }

                    case gr::lora::DecoderState::STOP: {
                        consume_each(upchirp71_len);
                        break;
                    }

                    default: {
                        std::cerr << "[LoRa Decoder] WARNING : No state! Shouldn't happen\n";
                        break;
                    }
                }
            }
           
            
            
            //std::cout<<"child81: "<<child81<<std::endl;
            if((lx_flag==0)&&(program_start_all_lx==true))
            {
                memcpy (p_map, input, sizeof(gr_complex) * numbles_for_SF); 

                //printf("fathertox71, start has send\n");
                write(file_fathertox711[OUTPUT], "start", 5);
                //write(file_fathertox721[OUTPUT], "start", 5);
                /*write(file_fathertox811[OUTPUT], "start", 5);
                write(file_fathertox821[OUTPUT], "start", 5);
                write(file_fathertox851[OUTPUT], "start", 5);
                write(file_fathertox911[OUTPUT], "start", 5);*/
                /*write(file_fathertox921[OUTPUT], "start", 5);
                write(file_fathertox951[OUTPUT], "start", 5);
                write(file_fathertox1021[OUTPUT], "start", 5);
                write(file_fathertox1051[OUTPUT], "start", 5);*/
                gettimeofday(&dwStart,NULL); 
                gr_complex result;
                //float auto_corr_D=xcorr(&input[0], &input[0], NULL,  numbles_for_SF,numbles_for_SF); 
                volk_32fc_x2_conjugate_dot_prod_32fc(&result, &input[0], &input[0], numbles_for_SF);
                float auto_corr_D=abs(result);

                auto_corr_D=auto_corr_D*10000;
                //std::cout<<"f:auto_cott_d: "<<auto_corr_D<<std::endl;
                //printf("father pid\n");
                memset (buf, ' ', sizeof(char) * (255));
                sprintf(buf,"%.4f",auto_corr_D);
                int i=0;
                for(i;i<255;i++){
                    if(buf[i]==' '){
                        break;
                    }  
                }

                //printf("fathertox71, D_auto has send %.4f\n",auto_corr_D);
                write(file_fathertox71[OUTPUT], buf, i); 
                //write(file_fathertox72[OUTPUT], buf, i);
                /*write(file_fathertox81[OUTPUT], buf, i);
                write(file_fathertox82[OUTPUT], buf, i);
                write(file_fathertox85[OUTPUT], buf, i);
                write(file_fathertox91[OUTPUT], buf, i);*/
                /*write(file_fathertox92[OUTPUT], buf, i);
                write(file_fathertox95[OUTPUT], buf, i);
                write(file_fathertox102[OUTPUT], buf, i);
                write(file_fathertox105[OUTPUT], buf, i);*/

                memset (buf, ' ', sizeof(char) * (255));
                returned_count=0;
                //printf("father wait dete_value\n");

                while(num_end_pro!=1){

                    if(!x71_flag){
                        returned_count = read(file_x71tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            //printf("received:  %s\n",buf);
                            int receive_part=1;
                            for(int i=0;i<255;i++){
                                //std::cout<<"buf:"<<i<<" "<<buf[i]<<std::endl;
                                if(*(buf+i)=='+'){
                                    receive_part=i;
                                    continue;
                                }
                                if(*(buf+i)=='#'){
                                    break;
                                }
                                if(receive_part==1){
                                    *(buf1+i)=*(buf+i);
                                }
                                else{
                                    *(buf2+i-receive_part)=*(buf+i);
                                }
                            }
                            //printf("received1:  %s\n",buf1);
                            //printf("received2:  %s\n",buf2);
                            lxdete_value[0]=atof(buf1);
                            start_dex71=atof(buf2);
                            //std::cout<<lxdete_value[0]<<" "<<start_dex71<<std::endl;
                            returned_count=0;
                            x71_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             1"<<std::endl;
                        }
                    }
                     if(!x72_flag){
                        returned_count = read(file_x72tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[1]=atof(buf);

                            returned_count=0;
                            x72_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             2"<<std::endl;
                            //sleep(60);
                        }
                    }
                  /*if(!x81_flag){
                        returned_count = read(file_x81tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[2]=atof(buf);
                            returned_count=0;
                            x81_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             3"<<std::endl;
                        }
                    }
                    if(!x82_flag){
                        returned_count = read(file_x82tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[3]=atof(buf);
                            returned_count=0;
                            x82_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             4"<<std::endl;
                        }
                    }
                    if(!x85_flag){
                        returned_count = read(file_x85tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[4]=atof(buf);
                            returned_count=0;
                            x85_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             5"<<std::endl;
                        }
                    }
                    if(!x91_flag){
                        returned_count = read(file_x91tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[5]=atof(buf);
                            returned_count=0;
                            x91_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             6"<<std::endl;
                        }
                    }*/
                   /*if(!x92_flag){
                        returned_count = read(file_x92tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[6]=atof(buf);
                            returned_count=0;
                            x92_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             7"<<std::endl;
                        }
                    }
                    if(!x95_flag){
                        returned_count = read(file_x95tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[7]=atof(buf);
                            returned_count=0;
                            x95_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             8"<<std::endl;
                        }
                    }
                     if(!x102_flag){
                        returned_count = read(file_x102tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[8]=atof(buf);
                            returned_count=0;
                            x102_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                            //std::cout<<"                                             9"<<std::endl;
                        }
                    }
                    if(!x105_flag){
                        returned_count = read(file_x105tofather[INPUT], buf, sizeof(buf));
                        if(returned_count!=0){
                            lxdete_value[9]=atof(buf);
                            returned_count=0;
                            x105_flag=true;
                            num_end_pro=num_end_pro+1;
                            memset (buf, ' ', sizeof(char) * (255));
                        }
                    }*/
                    //std::cout<<"num_end_pro: "<<num_end_pro<<std::endl;
                }
                //std::cout<<"lxdete_value[0]: "<<lxdete_value[0]<<std::endl;
                //samples_to_file("/home/lx/decode_lora/data71.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
                if((lxdete_value[0]>1.5)){
                //for(int i=0;i<10;i++){
                   // if(lxdete_value[i]>2){
                    std::cout<<"                 lxdete_value[0]: "<<lxdete_value[0]<<std::endl;
                    //std::cout<<"                 lxdete_value[1]: "<<lxdete_value[1]<<std::endl;
                    //std::cout<<"                 lxdete_value[2]: "<<lxdete_value[2]<<std::endl;
                    //std::cout<<"                 lxdete_value[3]: "<<lxdete_value[3]<<std::endl;
                    //std::cout<<"                 lxdete_value[4]: "<<lxdete_value[4]<<std::endl;
                    //std::cout<<"                 lxdete_value[5]: "<<lxdete_value[5]<<std::endl;
                    //std::cout<<"                 lxdete_value[6]: "<<lxdete_value[6]<<std::endl;
                    //std::cout<<"                 lxdete_value[7]: "<<lxdete_value[7]<<std::endl;
                    //std::cout<<"                 lxdete_value[8]: "<<lxdete_value[8]<<std::endl;
                    //std::cout<<"                 lxdete_value[9]: "<<lxdete_value[9]<<std::endl;
                        //break;
                    //lx_flag=1;
                    double time = usrp->get_time_now_to_real_secs(0);
					double newlx=0;
					double oldlx=0;
					for (int i=0;i<20;i++){
						newlx = usrp->get_time_now_to_real_secs(0);
						std::cout << boost::format("Now_Time::  %.9f seconds\n") % (newlx-oldlx)<<std::endl;
						oldlx=newlx;
					}
                    time=time+double(start_dex71/1000000.0);
					std::cout << boost::format("Now_Time::  %.9f seconds\n") % (time)<<std::endl;
					time_uart=0;
					time_uart=time;
                    //std::cout <<"start_uart"<<std::endl;
                    //write_uart(time);
                    //std::cout <<"end_uart"<<std::endl;
                    //time_to_file_add("/home/lx/decode_lora/time.txt",time);
                    
                    if(lx_flag==0){
                        //samples_to_file("/home/lx/decode_lora/data71.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
                        
                        consume_each(start_dex71-1024);
                        d_corr_fails = 0u;
                        //sleep(60);
                        
                    }
                    lx_flag=lx_flag+1;
                    /*if(lx_flag==2){
                        samples_to_file("/home/lx/decode_lora/data72.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
                        uint32_t count=(numbles_for_SF-numbles_for_SF%upchirp71_len)/upchirp71_len;
                        float* value=(float *) malloc(sizeof(float) * (count));
                        match_filter(&d_dnchirp71[0], &d_upchirp71[0], &input[0],value, count,upchirp71_len);
                        for(uint32_t i=0;i<count;i++){
                            std::cout<<"value:"<<value[i]<<std::endl;
                        }
                        consume_each(count*upchirp71_len);
                        //sleep(60);
                    }
                    if(lx_flag==3){
                        samples_to_file("/home/lx/decode_lora/data73.bin", &input[0], numbles_for_SF, sizeof(gr_complex));
                        uint32_t count=(numbles_for_SF-numbles_for_SF%upchirp71_len)/upchirp71_len;
                        float* value=(float *) malloc(sizeof(float) * (count));
                        match_filter(&d_dnchirp71[0],&d_upchirp71[0], &input[0],value, count,upchirp71_len);
                        for(uint32_t i=0;i<count;i++){
                            std::cout<<"value2:"<<value[i]<<std::endl;
                        }
                        sleep(60);
                    }*/
                //}
                }
                else{
                    //lx_flag=0;
                    consume_each(numbles_for_SF);
                }
                for (int i=0;i<numbers_para;i++){
                    lxdete_value[i]=0;
                }

                /*float auto_corr_D=xcorr(&input[0], &input[0], NULL,  numbles_for_SF,numbles_for_SF);
                int para_dex=0;
                float dete_value=correlate_dete(&input[0],auto_corr_D,&para_dex);
                if(dete_value>1.5){
                    std::cout<<"            dete_value: "<<dete_value<<" para_dex: "<<para_dex<<std::endl;
                }*/
                //consume_each(numbles_for_SF);

                gettimeofday(&dwEnd,NULL);  
                dwTime = 1000000*(dwEnd.tv_sec-dwStart.tv_sec)+(dwEnd.tv_usec-dwStart.tv_usec);  
               // printf("P_T: %ld\n",dwTime); 
            }
            /*else{
                consume_each(numbles_for_SF);
            }*/
            
            /*if(dete_noise){
                int para_dex=0;
                float dete_value=correlate_dete(&input[0],auto_corr_D,&para_dex);
                if(dete_value>1.5){
                    std::cout<<"            dete_value: "<<dete_value<<" para_dex: "<<para_dex<<std::endl;
                }
                 consume_each(numbles_for_SF);
            }
            gettimeofday(&dwEnd,NULL);  
            dwTime = 1000000*(dwEnd.tv_sec-dwStart.tv_sec)+(dwEnd.tv_usec-dwStart.tv_usec);  
            printf("P_T: %ld\n",dwTime); */
            
            
            /*if(lx_flag==1)
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
                            //std::cout<<"start_SYNC"<<std::endl;
                            detect_count=0;
                            break;
                        }
                        else{
                            detect_count++;
                        }
                        if(detect_count==5){
                            //lx_flag=0;
                            detect_count=0;
                            
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
                                                            
                            }
                            if (d_corr_fails > 4u) {
                                d_state = gr::lora::DecoderState::DETECT;
                                //lx_flag=0;
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
                            //lx_flag=0;
                            sleep(60);
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
            printf("HahA\n");
            //std::cerr << "[LoRa Decoder] WARNING : Setting the sample rate during execution is currently not supported." << std::endl
            //         << "Nothing set, kept SR of " << d_samples_per_second << "." << std::endl;
        }
        void decoder_impl::set_usrp_sptr(dev_sptr usrp_sprt) {
            
            if(usrp==NULL){
                printf("HahA1\n");
                usrp=usrp_sprt;
            }
            else{
                printf("HahA2\n");
                double time = usrp->get_time_now_to_real_secs(0);
                std::cout << boost::format("Now_Time::  %.9f seconds\n") % (time)<<std::endl;
                program_start_all_lx=true;
            }
            
            //printf("HahA2\n");
            //time.get_real_secs();
        }
        
    } /* namespace lora */
} /* namespace gr */
