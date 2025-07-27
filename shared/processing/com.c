/* Simple AM demodulator+processing (Dwaipayan Shikari)
atmega328p is used here a receiver , the receiver data is send to pc -->processed --->stored played
*/

#include <stdio.h> // standard stuffs
#include <stdlib.h>  // for sturct
#include <string.h> //memset 
#include <termios.h> //for the serial stuff https://www.man7.org/linux/man-pages/man3/termios.3.html
#include <fcntl.h> //for file descriptor
#include <unistd.h>// for the close function
#include <time.h>  // for timing operations

int main() {
  
    int fd = open("/dev/ttyUSB0", O_RDONLY | O_NOCTTY);  // i am using a linux system so /dev/ttyUSB , for windows use COM PORT and 
//windows api , for mac unix 
//open the incoming stream from serial (read-only) 

    if (fd < 0) { //if file descriptor returns negative value it has failed to open teh serial port (plug your device here Atmega328p)
        perror("open");
        return 1; //exit
    }

    //now set the baud rate to 115200 (optimal in this case)
    struct termios tty; // renamimg the struct as tty
    memset(&tty, 0, sizeof(tty)); // clearing all value to zero , or in either words clearing the serial buffer 
    tcgetattr(fd, &tty); // now link the driver with the file descriptor(serial stream will directly go into the file)
    cfsetispeed(&tty, B115200); // input , output speed of UART terminal is 115200
    cfsetospeed(&tty, B115200);
    tty.c_cflag |= (CLOCAL | CREAD);  // ignore modem control and enable the reciever 
    tty.c_cflag &= ~CSIZE;  // no size mask( actually i did not understand what it means , so i went to make it default to zero)
    tty.c_cflag |= CS8; // 8bit
    tty.c_cflag &= ~PARENB; // no parity 
    tty.c_cflag &= ~CSTOPB;  // one stop bit , not two
    tty.c_cflag &= ~CRTSCTS; // no hardware control flow , in simple uart not needed 
    tty.c_lflag = 0;    // these are control modes not in use  
    tty.c_oflag = 0;
    tty.c_iflag = 0;
    tcsetattr(fd, TCSANOW, &tty);  // now locking the settings of our serial driver ( just like KBC(kaun banega crorepati))

    //Here output is written in raw binary file , 'wb' means writing in binary
    FILE *output = fopen("audio.raw", "wb"); //
    if (!output) {  //if output address is null then will print "fopen not found"
        perror("fopen");
        close(fd); // close the descriptor
        return 1;
    }  
/*  DIGITAL FILTERNING PORTION*/
/* here i have used a simple low pass filter , according to nyquist we need atleast 2 times sampling frequency than intended playback
(from the actual results we got 5856 smaples/sec so if we could unaliased frequency upto 2.9 khz , for referenece for a standard youtue playback is at 
20khz and smapling frequnecy is generally kept at 44.1 khz , so yeah will we get a shitty voice 
(*/
    // Low-pass filter (5 kHz cutoff, 4.8 kHz playback)
    float alpha = 0.68; // 1 - 2 * pi * 5000 / 4800, (IIR filter)
    float y = 0; // Filter state
    unsigned char buf[128]; //this is the serial buffer ,  Buffer for multiple samples
    int sample;  // in atmega328p microcontroller i have programmed it to send 10 bits (in 8 bit +2bit) format for faster data transfer
// then we will merge those to in this sample variable
    short audio_buffer[64]; // This is our actual audio buffer,(RAW binary after modification and filtering)
    int buffer_index = 0; //useful to know when the buffer has filled 
    int sample_count = 0; //count of samples extracted 
    int read_count = 0; // sample value read from buffer (file --->buffer ---->read)
    int clip_count = 0;   // clip can occur due to pull up nature of the analogpin , means when no or weak signal then pin is generally pulled high via 
// internal pullup resistor
    time_t start_time = time(NULL); // from the time library count has 
    time_t end_time = start_time + 120; // Run for 2 minutes

    while (time(NULL) < end_time) {
        // Read serial data (2 bytes per sample)
        int n = read(fd, buf, 2);    //sample value read from buffer (file --->buffer ---->read)
        read_count++;       // after 2 buffer we get  buf 0 __1|__0|__1|__1|__0|__0|__1|__0| (8 bit)  (eg 0xB2)
                                                 //   buf 1 __0|__0|__0|__0|__0|__0|__0|__1| (2 bit)  (eg 0x01)
        if (n != 2) continue;
        // Combine bytes
        sample = (buf[0] << 8) | buf[1];
/* here the main philoshophy will be debug and storing the buffer in sample and debug the output */
        // Check for clipping
        if (sample >= 1023 || sample <= 0) {  // 1023 is 1.1v (1.1 v reference is selected in the ADC ) , clipping is extreme edges 
            clip_count++;
        }
        // let's continuosly see the value after each 100th time (these are the raw values not compatibel  for audio pipe still )
        if (sample_count % 100 == 0) {
            printf("Raw bytes: 0x%02x 0x%02x, Sample: %d, Rectified: %f\n", 
                   buf[0], buf[1], sample, (float)abs(sample));  // raw data then smaple and rectified
            fflush(stdout);  // fflush is use to force flush data from the FILE the raw binary values
        }
        // Demodulate: here it is happening via taking the abs value of the sample also giving it a  offset
// as for general in atmega328p adcis 10 bit so max value 1023 so for centering the value should be 512 
        float x = abs(sample-512); 
        y = alpha * y + (1 - alpha) * x; // Recursive low-pass filter  , y is the filtered value
        // Scale to 16-bit signed audio
        audio_buffer[buffer_index] = (short)(y * 64 - 16384); // short means 16 bit 2^6(y-2^8) now y in max be 2^10 
        buffer_index++;   
        sample_count++;
        // now we need to so write buffer when full we are chossing 64 
        if (buffer_index >= 64) {
            fwrite(audio_buffer, sizeof(short), buffer_index, output);
            buffer_index = 0;
        }
        //condition reset
        if (time(NULL) - start_time >= 1) {
            printf("Samples/sec: %d, Reads/sec: %d, Clips/sec: %d\n", 
                   sample_count, read_count, clip_count);
            sample_count = 0;
            read_count = 0;
            clip_count = 0;
            start_time = time(NULL);
        }
    }
    // Write remaining buffer
    if (buffer_index > 0) {
        fwrite(audio_buffer, sizeof(short), buffer_index, output);   //fwrite just writes binary stream of audio buffer , now OS will take care of to 
// write that into sound driver
    }

    fclose(output);
    close(fd);
    printf("Saved to audio.raw. Play with: aplay -f S16_LE -r 5800 audio.raw\n");
    return 0;
}