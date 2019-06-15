#include "ros/ros.h"
#include "complex.h"
#include "emotiv_msgs/Data.h"
#include "commands.h"

#define PI 3.14159265359
typedef std::complex<double> complex;

const int N = 128; // The size of the window for the mSDFT
const double Fs = 128.0; // Sampling frequency

// Index for the array that contains the last N samples
unsigned int idx = 0;

// Butterworth filter coefficients
float a[11] = {0.00327921630634217, 0, -0.0163960815317109, 0, 0.0327921630634217, 0, -0.0327921630634217, 0, 0.0163960815317109, 0, -0.00327921630634217};
float b[10] = {-6.72679683930659, 20.8674678136428, -39.468136495964, 50.5350124002961, -45.8189741005841, 29.7904061442053,  -13.7098300272873, 4.27400999194762, -0.815388394262081, 0.0723156691029586};

// This defines the frequency bins
const int startFreq = 6; // Lowest frequency bin
const int stopFreq = 18; // Highest frequency bin
const int nFreqs = (stopFreq - startFreq) * 10; // Number of frequency bins

// The number of frequency bin groups
const int arrSize = 5;

float freqzResult[arrSize] = {6.66, 8.6, 10, 12, 15}; // The frequencies looked at
int freqzIndx[arrSize] = {7, 26, 40, 60, 90}; // The indexes for the frequencies looked at

struct channel{
    // input signal
    double in[N];

    int counter; // A counter used to initialize the butterworth filter

    // Frequencies of input signal after mSDFT
    complex freqs[nFreqs];

    // these are just there to optimize (get rid of index lookups in sdft)
    double oldest_data, newest_data;

    // Array for inputsand outputs of the butterworth filter
    float inFil[11];
    float outFil[10];
    // Indexes for the above
    int inFilIndex;
    int outFilIndex;

    double powr[nFreqs]; // The power of the frequencies found

    channel(){
        // clear data
        for (int i = 0; i < N; ++i){
            in[i] = 0;
        }

        inFil[10] = 0;
        for(int i = 0; i < 10; i++){
            inFil[i] = 0;
            outFil[i] = 0;
        }
        inFilIndex = 0;
        outFilIndex = 0;

        counter = 0;

        oldest_data = newest_data = 0;
    }

    // The butterworth function
    float filt(float inp){
        inFil[inFilIndex] = inp; // New input
        counter++;

        if(counter < 11){ // If filter has not yet been initialized
            ++inFilIndex = (inFilIndex < 11 ? inFilIndex : 0);
            outFil[outFilIndex] = 0;
            ++outFilIndex = (outFilIndex < 10 ? outFilIndex : 0);
            return 0;
        }

        float outp = 0; // Output variable used to find output of filter

        // Calculates output based on last inputs
        for(int i = 0; i < 11; i++){
            outp += a[i] * inFil[inFilIndex];
            --inFilIndex = (inFilIndex >= 0 ? inFilIndex : 10);
        }
        ++inFilIndex = (inFilIndex < 11 ? inFilIndex : 0);

        // Calculates output based on last outputs
        for(int i = 0; i < 10; i++){
            outp -= b[i] * outFil[outFilIndex];
            --outFilIndex = (outFilIndex >= 0 ? outFilIndex : 9);
        }
        ++outFilIndex = (outFilIndex < 10 ? outFilIndex : 0);

        outFil[outFilIndex] = outp; // Saves output to array

        return outp;
    }

    void add_data(float inp)
    {
        inp = filt(inp); // Input is being redefined to its filtered version
        oldest_data = in[idx];
        newest_data = in[idx] = inp; // Oldest input is replaced by new input
    }
};

struct program {
    ros::NodeHandle n;

    ros::Subscriber sub;

    double power[nFreqs]; // The mean power from all EEG channels used
    double allPow; // The sum of the mean power of all EEG channels used

    double freqzVal[arrSize]; // The sums of the frequency bin groups
    double ms[arrSize]; // The percentage of each frequency bin group

    channel o1, o2; // The EEG channel used, although o2 is disabled later

    complex co[nFreqs]; // An array for the complex coefficients

    double r, rN; // The parameters for the modified part of the SDFT

    program() {
        sub = n.subscribe<emotiv_msgs::Data>("/emotiv_raw", 10, &program::new_data, this);

        // Initiate the coefficients
        for(int i = 0; i < nFreqs; i++){
            double a = 2.0 * PI * (i * 0.1 + startFreq) / Fs;
            co[i] = complex(cos(a), sin(a));
            power[i] = 0;
        }

        // initialization of the modified parameters
        r = 0.997;
        rN = pow(r, N);
    }

    // The mSDFT function
    void sdft(channel &ch) {
        static int counter = 0; // A counter for initializing the mSDFT

        if(counter < N){ // The initializing part
            counter++;
            for (int i = 0; i < nFreqs; ++i) {
                ch.freqs[i] = r * (ch.freqs[i] + ch.newest_data) * co[i];
            }
            return;
        }

        // The actual mSDFT part
        complex delta = ch.newest_data - rN * ch.oldest_data;
        for (int i = 0; i < nFreqs; ++i) {
            ch.freqs[i] = r * (ch.freqs[i] + delta) * co[i]; // The updated bin is calculaed
            ch.powr[i] = abs(ch.freqs[i]/double(N)); // The power is calculated
        }
    }

    void SendCommand(channel &ch) {
        static Command cmd;

        // The code finds the index for the group with the highest percentage that also exceeds the threshold of 0.071
        int indx = -1;
        double tempMs = 0;

        for(int i = 0; i < arrSize; i++) {
            freqzVal[i] = ch.powr[freqzIndx[i] - 1] + ch.powr[freqzIndx[i]] + ch.powr[freqzIndx[i] + 1];
            ms[i] = freqzVal[i] / allPow;

            if(ms[i] > tempMs && ms[i] > 0.071) {
                tempMs = ms[i];
                indx = i;
            }
        }

        if(indx != -1)
            cmd.publish(indx); // The found index is published, if any was found
    }

    void new_data(const emotiv_msgs::Data::ConstPtr& msg){ // This is the callback function when a new sample has been received
        o1.add_data(msg->channel.O1.data); // The new sample is put through the filter

        sdft(o1); // The mSDFT is being calculated

        allPow = 0; // The allPow is reset so that it can be recalculated


        for(unsigned int i = 0; i < nFreqs; i++)
            allPow += o1.powr[i]; // The allPow is calculated by summing up 

        SendCommand(o1);

        ++idx = (idx < N ? idx : 0);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "signal_processing");

    program p;

    ros::spin();

    return 0;
}
