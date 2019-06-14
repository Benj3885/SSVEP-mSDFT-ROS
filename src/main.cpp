#include "ros/ros.h"
#include "complex.h"
#include "emotiv_msgs/Data.h"
#include "commands.h"

#define PI 3.14159265359

const int N = 128;

// index for input and output signals
unsigned int idx = 0;

float a[11] = {0.00327921630634217, 0, -0.0163960815317109, 0, 0.0327921630634217, 0, -0.0327921630634217, 0, 0.0163960815317109, 0, -0.00327921630634217};
float b[10] = {-6.72679683930659, 20.8674678136428, -39.468136495964, 50.5350124002961, -45.8189741005841, 29.7904061442053,  -13.7098300272873, 4.27400999194762, -0.815388394262081, 0.0723156691029586};

typedef std::complex<double> complex;


const int startFreq = 6;
const int stopFreq = 18;
const int nFreqs = (stopFreq - startFreq) * 10;
const double Fs = 128.0;

const int arrSize = 5;

float freqzResult[arrSize] = {6.66, 8.6, 10, 12, 15};
int freqzIndx[arrSize] = {7, 26, 40, 60, 90};

struct channel{
    // input signal
    double in[N];

    int counter;

    // frequencies of input signal after ft
    // Size increased by one because the optimized sdft code writes data to freqs[N]
    complex freqs[nFreqs];

    // these are just there to optimize (get rid of index lookups in sdft)
    double oldest_data, newest_data;

    float inFil[11];
    float outFil[10];
    int inFilIndex;
    int outFilIndex;

    double powr[nFreqs];

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

    float filt(float inp){
        inFil[inFilIndex] = inp;
        counter++;

        if(counter < 11){
            ++inFilIndex = (inFilIndex < 11 ? inFilIndex : 0);
            outFil[outFilIndex] = 0;
            ++outFilIndex = (outFilIndex < 10 ? outFilIndex : 0);
            return 0;
        }

        float outp = 0;

        for(int i = 0; i < 11; i++){
            outp += a[i] * inFil[inFilIndex];
            --inFilIndex = (inFilIndex >= 0 ? inFilIndex : 10);
        }
        ++inFilIndex = (inFilIndex < 11 ? inFilIndex : 0);

        for(int i = 0; i < 10; i++){
            outp -= b[i] * outFil[outFilIndex];
            --outFilIndex = (outFilIndex >= 0 ? outFilIndex : 9);
        }
        ++outFilIndex = (outFilIndex < 10 ? outFilIndex : 0);

        outFil[outFilIndex] = outp;

        return outp;
    }

    void add_data(float inp)
    {
        inp = filt(inp);
        oldest_data = in[idx];
        newest_data = in[idx] = inp;
    }
};

struct program {
    ros::NodeHandle n;

    ros::Subscriber sub;

    double power[nFreqs];
    double allPow;

    double freqzVal[arrSize];
    double ms[arrSize];

    channel o1, o2;

    complex co[nFreqs];

    double r, rN;

    program() {
        sub = n.subscribe<emotiv_msgs::Data>("/emotiv_raw", 10, &program::new_data, this);

        // Initiate the coefficients
        for(int i = 0; i < nFreqs; i++){
            double a = 2.0 * PI * (i * 0.1 + startFreq) / Fs;
            co[i] = complex(cos(a), sin(a));
            power[i] = 0;
        }

        r = 0.997;
        rN = pow(r, N);
    }

    void sdft(channel &ch) {
        static int counter = 0;

        if(counter < N){
            counter++;
            for (int i = 0; i < nFreqs; ++i) {
                ch.freqs[i] = r * (ch.freqs[i] + ch.newest_data) * co[i];
            }
            return;
        }

        complex delta =  - rN * ch.oldest_data;
        for (int i = 0; i < nFreqs; ++i) {
            ch.freqs[i] = r * (ch.freqs[i] + delta) * co[i];
            ch.powr[i] = abs(ch.freqs[i]/double(N));
        }
    }

    void SendCommand(channel &ch) {
        static Command cmd;

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
            cmd.publish(indx);
    }

    void new_data(const emotiv_msgs::Data::ConstPtr& msg){
        o1.add_data(msg->channel.O1.data);

        sdft(o1);

        allPow = 0;


        for(unsigned int i = 0; i < nFreqs; i++)
            allPow += o1.powr[i];

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
