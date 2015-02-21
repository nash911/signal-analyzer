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
        std::cerr << "SignalAnalyzer Error: SignalAnalyzer class." << std::endl
                  << "main(int, char*) method" << std::endl
                  << "Too many or too few command line parameters: "<< argc << std::endl;

        exit(1);
    }

    Signal_Analyzer_List sList(fileName);
    sList.filter_signal();
    sList.show_signal_amplitudes();
    sList.show_signal_offsets();
    sList.calculate_phaseVector();
    sList.show_phase_relation();
    sList.show_signal_frequency();

    return 0;
}
