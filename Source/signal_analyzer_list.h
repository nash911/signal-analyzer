/********************************************************************************************/
/*                                                                                          */
/*   SignalAnalyzer: A project for extracting signal parameters from raw signal data.       */
/*                                                                                          */
/*   S I G N A L   A N A L Y Z E R   L I S T   C L A S S   H E A D E R                      */
/*                                                                                          */
/*   Avinash Ranganath                                                                      */
/*   Robotics Lab, Department of Systems Engineering and Automation                         */
/*   University Carlos III of Mardid(UC3M)                                                  */
/*   Madrid, Spain                                                                          */
/*   E-mail: nash911@gmail.com                                                              */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                         */
/*                                                                                          */
/********************************************************************************************/

#ifndef SIGNAL_ANALYZER_LIST_H
#define SIGNAL_ANALYZER_LIST_H

#include "signal_analyzer.h"

#define FILTER_EPOCH 1

class Signal_Analyzer_List
{
public:
    Signal_Analyzer_List(const char*);
    Signal_Analyzer_List(const char*, const vector<unsigned int>&);

    unsigned int get_num_signals(const char* const signalsFileName) const;

    void filter_signal(void);
    void crop_signal_length(const double, const double);
    void show_signal_amplitudes(void);
    void show_signal_offsets(void);
    void calculate_phaseVector_crest(void);
    void calculate_phaseVector_trough(void);
    void show_phase_relation_crest(void);
    void show_phase_relation_trough(void);
    void show_signal_frequency(void);
    void show_signal_range(void);

private:
    unsigned int no_of_signals;
    vector<Signal_Analyzer> S;

};


#endif // SIGNAL_ANALYZER_LIST_H
