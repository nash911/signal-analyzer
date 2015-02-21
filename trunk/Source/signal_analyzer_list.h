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

  unsigned int get_num_signals(const char* const signalsFileName) const;

  void filter_signal(void);
  void show_signal_amplitudes(void);
  void show_signal_offsets(void);
  void calculate_phaseVector(void);
  void show_phase_relation(void);
  void show_signal_frequency(void);

private:
  unsigned int no_of_signals;
  std::vector<Signal_Analyzer> S;

};


#endif // SIGNAL_ANALYZER_LIST_H
