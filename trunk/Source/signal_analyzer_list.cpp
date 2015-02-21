/********************************************************************************************/
/*                                                                                          */
/*   SignalAnalyzer: A project for extracting signal parameters from raw signal data.       */
/*                                                                                          */
/*   S I G N A L   A N A L Y Z E R   L I S T   C L A S S                                    */
/*                                                                                          */
/*   Avinash Ranganath                                                                      */
/*   Robotics Lab, Department of Systems Engineering and Automation                         */
/*   University Carlos III of Mardid(UC3M)                                                  */
/*   Madrid, Spain                                                                          */
/*   E-mail: nash911@gmail.com                                                              */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                         */
/*                                                                                          */
/********************************************************************************************/

#include"signal_analyzer_list.h"

// CONSTRUCTOR

/// Creates a Signal Analyzer List object
/// Number of signals on file is extracted from the data file.
/// Creates and adds to the list, the necessary numbe of Signal Analyzer objects.
/// Removes pre-existing output files.
/// @param signalsFileName Path and name of the file containing signals data.

Signal_Analyzer_List::Signal_Analyzer_List(const char* signalsFileName)
{
    //--Remove old output files--//
    system("exec rm -r ../Output/*");

    if(!signalsFileName)
    {
        signalsFileName = "../signals.dat";
    }

    no_of_signals = get_num_signals(signalsFileName);
    if(!no_of_signals)
    {
        std::cerr << "SignalAnalyzer Error: SignalAnalyzerList class." << std::endl
                  << "Signal_Analyzer_List(const char*) method" << std::endl
                  << "No signals found on file: "<< signalsFileName  << std::endl;

        exit(1);
    }

    //--Create a Signal Analyzer object per signal and add to the list--//
    for(unsigned int i=0; i<no_of_signals; i++)
    {
      Signal_Analyzer s_an(signalsFileName, i+1);
      S.push_back(s_an);
    }
}


// CONSTRUCTOR

/// Creates a Signal Analyzer List object
/// Ids of the signals to be extracted from the file is accepted as a vector of IDs.
/// Creates and adds to the list, the necessary numbe of Signal Analyzer objects.
/// Removes pre-existing output files.
/// @param signalsFileName Path and name of the file containing signals data.
/// @param signalIDList A vector containing the IDs of the signals to be extracted from the file.

Signal_Analyzer_List::Signal_Analyzer_List(const char* signalsFileName, const std::vector<unsigned int>& signalIDList)
{
    //--Remove old output files--//
    system("exec rm -r ../Output/*");

    if(!signalsFileName)
    {
        signalsFileName = "../signals.dat";
    }

    no_of_signals = signalIDList.size();
    if(!no_of_signals)
    {
        std::cerr << "SignalAnalyzer Error: SignalAnalyzerList class." << std::endl
                  << "Signal_Analyzer_List(const char*, , const std::vector<unsigned int>&) method" << std::endl
                  << "No signals found on file: "<< signalsFileName  << std::endl;

        exit(1);
    }

    //--Create a Signal Analyzer object per signal and add to the list--//
    for(unsigned int i=0; i<no_of_signals; i++)
    {
      Signal_Analyzer s_an(signalsFileName, signalIDList[i]);
      S.push_back(s_an);
    }
}


// unsigned int get_num_signals(const char* const) method

/// This method extracts and returns the number of signals on the data file.
/// @param signalsFileName Path and name of the file containing signals data.

unsigned int Signal_Analyzer_List::get_num_signals(const char* const signalsFileName) const
{
    double num_signals = 0;

    std::fstream inputFile;
    inputFile.open(signalsFileName, std::ios::in);

    if(!inputFile.is_open())
    {
      std::cerr << "SignalAnalyzer Error: SignalAnalyzerList class." << std::endl
                << "get_num_signals(const char* const) method" << std::endl
                << "Cannot open Parameter file: "<< signalsFileName  << std::endl;

      exit(1);
    }

    std::string line;
    double dNum;
    std::size_t found;

    //--Omitting lines containing '#'--//
    do
    {
        getline(inputFile, line);
        found = line.find("#");
    }while(found != std::string::npos);

    std::stringstream ssLine(line);
    ssLine >> dNum;

    //--Extracting the number of signals on file--//
    while(ssLine >> dNum)
    {
        num_signals++;
    }


    std::cout << std::endl << "Number of signals on file: " << num_signals << std::endl << std::endl;
    inputFile.close();

    return num_signals;
}


// void filter_signal(void) method

/// This method filters crests and troughs of all the signals in the list.
/// Each signal is filteres by taking as reference, all other signals in the list, once.
/// So each signal is filtered n-1 times, where n is the total number of signals.

void Signal_Analyzer_List::filter_signal()
{
    //--Filter signal crests and troughs individualy, by comparing each signal with the rest of the signals--//
    for(unsigned int n=0; n<FILTER_EPOCH; n++)
    {
        for(unsigned int i=0; i<no_of_signals; i++)
        {
            for(unsigned int j=0; j<no_of_signals; j++)
            {
                if(i != j)
                {
                    S[i].filter_signal_crest(S[j]);
                    S[i].filter_signal_trough(S[j]);
                }
            }
        }
    }

    //--Show the number of Crests and Troughs found, for each signal, after filtering--/
    for(unsigned int i=0; i<no_of_signals; i++)
    {
        std::cout << "Signal_" << S[i].get_signal_id() << ": No. of Crests found = " << S[i].get_crest_time_size()
                  << "   No. of Trough found = " << S[i].get_trough_time_size() << std::endl;
    }
}


// void show_signal_amplitudes(void) method

/// Estimates and displays average amplitude of all the signals on the list.

void Signal_Analyzer_List::show_signal_amplitudes(void)
{
    std::cout << std::endl << "         Signal Amplitudes" << std::endl;
    for(unsigned int i=0; i<no_of_signals; i++)
    {
        std::cout << "Signal_" << S[i].get_signal_id() << ": " << S[i].estimate_amplitude() << std::endl;
    }
}


// void show_signal_offset(void) method

/// Estimates and displays average offset of all the signals on the list.

void Signal_Analyzer_List::show_signal_offsets(void)
{
    std::cout << std::endl << "         Signal Offsets" << std::endl;
    for(unsigned int i=0; i<no_of_signals; i++)
    {
        std::cout << "Signal_" << S[i].get_signal_id() << ": " << S[i].estimate_offset() << std::endl;
    }
}


// void calculate_phaseVector(void) method

/// Esimates phase difference between all pairs of signals on the list, as a vector,
/// and then stores the same on a plottable file.

void Signal_Analyzer_List::calculate_phaseVector(void)
{
    std::vector<std::vector<std::vector<double> > > phase;
    std::vector<double> time;

    std::fstream phaseFile_180;
    std::fstream phaseFile_360;

    //--Calculate phase difference between signals, for every pair of signals.
    for(unsigned int i=0; i<no_of_signals-1; i++)
    {
        for(unsigned int j=i+1; j<no_of_signals; j++)
        {
            phase.push_back(S[i].calculate_phase_crest(S[j]));
        }
    }

    phaseFile_180.open("../Output/phase180.dat", std::ios::out);
    phaseFile_360.open("../Output/phase360.dat", std::ios::out);

    //--Accumulate all unique time values from all the phase vectors.
    for(unsigned int i=0; i<phase.size(); i++)
    {
        for(unsigned int j=0; j<phase[i].size(); j++)
        {
            if(std::find(time.begin(), time.end(), phase[i][j][0])==time.end())
            {
                time.push_back(phase[i][j][0]);
            }
        }
    }

    //--Sort the time vector in accending order.
    std::sort(time.begin(), time.end());

    //--Search the phase vectors and store the phase value closest to each time value, in phase graph file.
    for(unsigned int n=0; n<time.size(); n++)
    {
        phaseFile_180 << time[n];
        phaseFile_360 << time[n];

        for(unsigned int i=0; i<phase.size(); i++)
        {
            for(unsigned int j=0; j<phase[i].size(); j++)
            {
                if(phase[i][j][0] >= time[n])
                {
                    phaseFile_180 << " " << phase[i][j][1];
                    phaseFile_360 << " " << phase[i][j][2];

                    break;
                }
            }
        }

        phaseFile_180 << std::endl;
        phaseFile_360 << std::endl;
    }

    phaseFile_180.close();
    phaseFile_360.close();
}



// void show_signal_phase_relation(void) method

/// Estimates and displays average phase difference between all pairs of signals on the list.

void Signal_Analyzer_List::show_phase_relation(void)
{
    double avg_phase_diff;

    std::cout << std::endl << "         Phase difference between pairs of signals (-180°, 180°]" << std::endl;

    //--Calculate phase difference between signals, for every pair of signals.
    for(unsigned int i=0; i<no_of_signals-1; i++)
    {
        for(unsigned int j=i+1; j<no_of_signals; j++)
        {
            std::vector<std::vector<double> > phaseVector;
            std::vector<double> phaseRelation;

            phaseVector = S[i].calculate_phase_crest(S[j]);

            for(unsigned int k=0; k<phaseVector.size(); k++)
            {
                phaseRelation.push_back(phaseVector[k][1]);
            }
            avg_phase_diff = std::accumulate(phaseRelation.begin(), phaseRelation.end(), 0.0)/phaseRelation.size();

            std::cout << "Signal_" << S[i].get_signal_id() << " AND Signal_"
                      << S[j].get_signal_id() << ": " << avg_phase_diff << "°" << std::endl;
        }
    }

}


// void show_signal_frequency(void) method

/// Estimates and displays average frequency of all the signals on the list.

void Signal_Analyzer_List::show_signal_frequency(void)
{
    std::vector<double> frequency;
    double avg_freq;

    for(unsigned int i=0; i<no_of_signals; i++)
    {
        frequency.push_back(S[i].estimate_frequency());
    }
    avg_freq = std::accumulate(frequency.begin(), frequency.end(), 0.0)/frequency.size();

    std::cout << std::endl << "Signal Frequency: " << avg_freq << std::endl;
}
