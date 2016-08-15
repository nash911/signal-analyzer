#include "signal_analyzer_list.h"


int main(int argc, char* argv[])
{
    char* fileName;

    if(argc == 1)
    {
        fileName = NULL;
    }
    else if(argc == 2)
    {
        fileName = argv[1];
    }
    else
    {
        cerr << "SignalAnalyzer Error: SignalAnalyzer class." << endl
             << "main(int, char*) method" << endl
             << "Too many or too few command line parameters: "<< argc << endl;

        exit(1);
    }

    Signal_Analyzer_List sList(fileName);
    sList.filter_signal();
    sList.crop_signal_length(1.0, 119.0);
    sList.show_signal_amplitudes();
    sList.show_signal_offsets();
    sList.show_signal_range();
    sList.calculate_phaseVector();
    sList.show_phase_relation();
    sList.show_signal_frequency();

    return 0;
}
