
#include <vector>
#include <cmath>
#include <algorithm>
#include <Arduino.h>

#ifndef QUAD_FILTER_H
#define QUAD_FILTER_H
// Arduino-compatible string type

void sendLine(const String &line);

// Arduino-compatible math functions
#ifdef ARDUINO
#define M_PI 3.14159265358979323846
#endif

// Arduino-compatible dynamic array template
template<typename T>
class Vector {
private:
    T* data;
    size_t capacity;
    size_t size_;
    
public:
    Vector() : data(nullptr), capacity(0), size_(0) {}
    
    explicit Vector(size_t n) : data(nullptr), capacity(0), size_(0) {
        resize(n);
    }
    
    Vector(size_t n, const T& val) : data(nullptr), capacity(0), size_(0) {
        resize(n);
        for (size_t i = 0; i < n; ++i) {
            data[i] = val;
        }
    }
    
    ~Vector() {
        delete[] data;
    }
    
    // Copy constructor
    Vector(const Vector& other) : data(nullptr), capacity(0), size_(0) {
        *this = other;
    }
    
    // Assignment operator
    Vector& operator=(const Vector& other) {
        if (this != &other) {
            delete[] data;
            size_ = other.size_;
            capacity = other.capacity;
            if (capacity > 0) {
                data = new T[capacity];
                for (size_t i = 0; i < size_; ++i) {
                    data[i] = other.data[i];
                }
            } else {
                data = nullptr;
            }
        }
        return *this;
    }
    
    void resize(size_t n) {
        if (n > capacity) {
            reserve(n);
        }
        size_ = n;
    }
    
    void reserve(size_t n) {
        if (n > capacity) {
            T* newData = new T[n];
            for (size_t i = 0; i < size_; ++i) {
                newData[i] = data[i];
            }
            delete[] data;
            data = newData;
            capacity = n;
        }
    }
    
    void push_back(const T& val) {
        if (size_ >= capacity) {
            reserve(capacity == 0 ? 1 : capacity * 2);
        }
        data[size_++] = val;
    }
    
    T& operator[](size_t i) { return data[i]; }
    const T& operator[](size_t i) const { return data[i]; }
    
    size_t size() const { return size_; }
    bool empty() const { return size_ == 0; }
    
    T* begin() { return data; }
    T* end() { return data + size_; }
    const T* begin() const { return data; }
    const T* end() const { return data + size_; }
};

// Arduino-compatible min/max/clamp functions
template<typename T>
T min_val(const T& a, const T& b) {
    return (a < b) ? a : b;
}

template<typename T>
T max_val(const T& a, const T& b) {
    return (a > b) ? a : b;
}

template<typename T>
T clamp_val(const T& val, const T& min_v, const T& max_v) {
    return max_val(min_v, min_val(val, max_v));
}

// Arduino-compatible min/max element functions
template<typename T>
T min_element(const Vector<T>& vec) {
    if (vec.empty()) return T{};
    T minVal = vec[0];
    for (size_t i = 1; i < vec.size(); ++i) {
        if (vec[i] < minVal) minVal = vec[i];
    }
    return minVal;
}

template<typename T>
T max_element(const Vector<T>& vec) {
    if (vec.empty()) return T{};
    T maxVal = vec[0];
    for (size_t i = 1; i < vec.size(); ++i) {
        if (vec[i] > maxVal) maxVal = vec[i];
    }
    return maxVal;
}

// Arduino-compatible complex number class
class Complex {
public:
    double real, imag;
    
    Complex(double r = 0.0, double i = 0.0) : real(r), imag(i) {}
    
    Complex operator+(const Complex& other) const {
        return Complex(real + other.real, imag + other.imag);
    }
    
    Complex operator-(const Complex& other) const {
        return Complex(real - other.real, imag - other.imag);
    }
    
    Complex operator*(const Complex& other) const {
        return Complex(real * other.real - imag * other.imag,
                      real * other.imag + imag * other.real);
    }
    
    Complex operator/(const Complex& other) const {
        double denom = other.real * other.real + other.imag * other.imag;
        return Complex((real * other.real + imag * other.imag) / denom,
                      (imag * other.real - real * other.imag) / denom);
    }
    
    double magnitude() const {
        return sqrt(real * real + imag * imag);
    }
    
    double phase() const {
        return atan2(imag, real);
    }
};

// Arduino-compatible exponential function for complex numbers
Complex complex_exp(const Complex& z) {
    double r = exp(z.real);
    return Complex(r * cos(z.imag), r * sin(z.imag));
}

enum class FilterType {
    LOW_PASS,
    HIGH_PASS,
    BAND_PASS,
    BAND_STOP,
    PEAK,
    LOW_SHELF,
    HIGH_SHELF
};

class BiquadFilter {
private:
    // Filter coefficients
    double a0, a1, a2, b0, b1, b2;
    
    // Delay line (history)
    double x1, x2; // Input history
    double y1, y2; // Output history
    
    // Filter parameters
    double sampleRate;
    double frequency;
    double Q;
    double gain; // For shelving and peaking filters
    FilterType type;
    
    void calculateCoefficients() {
        double w = 2.0 * M_PI * frequency / sampleRate;
        double cosw = cos(w);
        double sinw = sin(w);
        double alpha = sinw / (2.0 * Q);
        double A = pow(10.0, gain / 40.0); // Convert dB to linear
        double beta = sqrt(A) / Q;
        
        switch (type) {
            case FilterType::LOW_PASS:
                b0 = (1.0 - cosw) / 2.0;
                b1 = 1.0 - cosw;
                b2 = (1.0 - cosw) / 2.0;
                a0 = 1.0 + alpha;
                a1 = -2.0 * cosw;
                a2 = 1.0 - alpha;
                break;
                
            case FilterType::HIGH_PASS:
                b0 = (1.0 + cosw) / 2.0;
                b1 = -(1.0 + cosw);
                b2 = (1.0 + cosw) / 2.0;
                a0 = 1.0 + alpha;
                a1 = -2.0 * cosw;
                a2 = 1.0 - alpha;
                break;
                
            case FilterType::BAND_PASS:
                b0 = alpha;
                b1 = 0.0;
                b2 = -alpha;
                a0 = 1.0 + alpha;
                a1 = -2.0 * cosw;
                a2 = 1.0 - alpha;
                break;
                
            case FilterType::BAND_STOP:
                b0 = 1.0;
                b1 = -2.0 * cosw;
                b2 = 1.0;
                a0 = 1.0 + alpha;
                a1 = -2.0 * cosw;
                a2 = 1.0 - alpha;
                break;
                
            case FilterType::PEAK:
                b0 = 1.0 + alpha * A;
                b1 = -2.0 * cosw;
                b2 = 1.0 - alpha * A;
                a0 = 1.0 + alpha / A;
                a1 = -2.0 * cosw;
                a2 = 1.0 - alpha / A;
                break;
                
            case FilterType::LOW_SHELF:
                b0 = A * ((A + 1.0) - (A - 1.0) * cosw + beta * sinw);
                b1 = 2.0 * A * ((A - 1.0) - (A + 1.0) * cosw);
                b2 = A * ((A + 1.0) - (A - 1.0) * cosw - beta * sinw);
                a0 = (A + 1.0) + (A - 1.0) * cosw + beta * sinw;
                a1 = -2.0 * ((A - 1.0) + (A + 1.0) * cosw);
                a2 = (A + 1.0) + (A - 1.0) * cosw - beta * sinw;
                break;
                
            case FilterType::HIGH_SHELF:
                b0 = A * ((A + 1.0) + (A - 1.0) * cosw + beta * sinw);
                b1 = -2.0 * A * ((A - 1.0) + (A + 1.0) * cosw);
                b2 = A * ((A + 1.0) + (A - 1.0) * cosw - beta * sinw);
                a0 = (A + 1.0) - (A - 1.0) * cosw + beta * sinw;
                a1 = 2.0 * ((A - 1.0) - (A + 1.0) * cosw);
                a2 = (A + 1.0) - (A - 1.0) * cosw - beta * sinw;
                break;
        }
        
        // Normalize coefficients
        b0 /= a0;
        b1 /= a0;
        b2 /= a0;
        a1 /= a0;
        a2 /= a0;
        a0 = 1.0;
    }
    
public:
    BiquadFilter(double sampleRate = 44100.0, double frequency = 1000.0, 
                 double Q = 0.707, FilterType type = FilterType::LOW_PASS, 
                 double gain = 0.0) 
        : sampleRate(sampleRate), frequency(frequency), Q(Q), type(type), gain(gain),
          x1(0.0), x2(0.0), y1(0.0), y2(0.0) {
        calculateCoefficients();
    }
    
    // Process single sample
    double process(double input) {
        double output = b0 * input + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
        
        // Update delay line
        x2 = x1;
        x1 = input;
        y2 = y1;
        y1 = output;
        
        return output;
    }
    
    // Process buffer of samples
    void processBuffer(const Vector<double>& input, Vector<double>& output) {
        output.resize(input.size());
        for (size_t i = 0; i < input.size(); ++i) {
            output[i] = process(input[i]);
        }
    }
    
    // Configuration methods
    void setFrequency(double freq) {
        frequency = clamp_val(freq, 20.0, sampleRate / 2.0);
        calculateCoefficients();
    }
    
    void setQ(double q) {
        Q = clamp_val(q, 0.1, 30.0);
        calculateCoefficients();
    }
    
    void setGain(double g) {
        gain = clamp_val(g, -40.0, 40.0);
        calculateCoefficients();
    }
    
    void setType(FilterType t) {
        type = t;
        calculateCoefficients();
    }
    
    void setSampleRate(double sr) {
        sampleRate = sr;
        calculateCoefficients();
    }
    
    // Reset filter state
    void reset() {
        x1 = x2 = y1 = y2 = 0.0;
    }
    
    // Get filter parameters
    double getFrequency() const { return frequency; }
    double getQ() const { return Q; }
    double getGain() const { return gain; }
    FilterType getType() const { return type; }
    double getSampleRate() const { return sampleRate; }
    
    // Get frequency response at a given frequency
    void getFrequencyResponse(double freq, double& magnitude, double& phase) const {
        double w = 2.0 * M_PI * freq / sampleRate;
        Complex z = complex_exp(Complex(0, w));
        Complex z2 = z * z;
        
        Complex num = Complex(b0) + Complex(b1) * z + Complex(b2) * z2;
        Complex den = Complex(a0) + Complex(a1) * z + Complex(a2) * z2;
        Complex H = num / den;
        
        magnitude = 20.0 * log10(H.magnitude());
        phase = H.phase() * 180.0 / M_PI;
    }
};

// Multi-stage filter for steeper rolloff
class CascadedFilter {
private:
    Vector<BiquadFilter> stages;
    
public:
    CascadedFilter(int numStages, double sampleRate = 44100.0, 
                   double frequency = 1000.0, double Q = 0.707, 
                   FilterType type = FilterType::LOW_PASS) {         
        stages.resize(numStages);
        for (int i = 0; i < numStages; ++i) {
            stages[i] = BiquadFilter(sampleRate, frequency, Q, type);
        }
    }

    double process(double input) {
        double output = input;
        for (size_t i = 0; i < stages.size(); ++i) {
            output = stages[i].process(output);
        }
        return output;
    }
    
    void processBuffer(const Vector<double>& input, Vector<double>& output) {
        output = input;
        for (size_t i = 0; i < stages.size(); ++i) {
            stages[i].processBuffer(output, output);
        }
    }
    
    double fre=0;

    void setF(double fr)
    {
        fre = fr;
    }

    void setFrequency(double freq) {
        setF(freq);
        for (size_t i = 0; i < stages.size(); ++i) {
            stages[i].setFrequency(freq);
        }
    }
    
    void setQ(double q) {
        for (size_t i = 0; i < stages.size(); ++i) {
            stages[i].setQ(q);
        }
    }

    double getQ()
    {
        return stages[0].getQ();
    }

    double getFrequency()
    {
         return stages[0].getFrequency();
    }
    

    void reset() {
        for (size_t i = 0; i < stages.size(); ++i) {
            stages[i].reset();
        }
    }
    
    size_t getStageCount() const { return stages.size(); }

};

  
#endif