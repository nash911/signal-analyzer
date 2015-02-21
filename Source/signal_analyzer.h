/********************************************************************************************/
/*                                                                                          */
/*   SignalAnalyzer: A project for extracting signal parameters from raw signal data.       */
/*                                                                                          */
/*   S I G N A L   A N A L Y Z E R   C L A S S   H E A D E R                                */
/*                                                                                          */
/*   Avinash Ranganath                                                                      */
/*   Robotics Lab, Department of Systems Engineering and Automation                         */
/*   University Carlos III of Mardid(UC3M)                                                  */
/*   Madrid, Spain                                                                          */
/*   E-mail: nash911@gmail.com                                                              */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                         */
/*                                                                                          */
/********************************************************************************************/


#ifndef SIGNAL_ANALYZER_H
#define SIGNAL_ANALYZER_H

#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include <stdlib.h>
#include <math.h>
#include <numeric>
#include <algorithm>

#define NOISE_THRESHOLD 5.0

class Signal_Analyzer
{
public:
    Signal_Analyzer(const char* const, const unsigned int);
    void extract_signal_data_from_file(const char* const);

    bool is_previous_signal_lower(const unsigned int) const;
    bool is_previous_signal_higher(const unsigned int) const;

    void select_signal_crest(void);
    void select_signal_trough(void);

    void filter_signal_crest(const Signal_Analyzer&);
    void filter_signal_trough(const Signal_Analyzer&);

    double estimate_amplitude(void) const;
    double estimate_offset(void) const;
    double estimate_frequency(void) const;
    std::vector<std::vector<double> > calculate_phase_crest(const Signal_Analyzer&) const;

    double get_crest_time(const unsigned int) const;
    double get_trough_time(const unsigned int) const;

    unsigned int get_crest_time_size(void) const;
    unsigned int get_trough_time_size(void) const;

    unsigned int get_signal_id(void) const;

private:
    unsigned int signal_id;
    double signal_mean;

    std::vector<double> time;
    std::vector<double> signal;

    std::vector<double> signal_crest;
    std::vector<double> signal_crest_time;

    std::vector<double> signal_trough;
    std::vector<double> signal_trough_time;
};

#endif // SIGNAL_ANALYZER_H
