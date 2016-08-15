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
        signalsFileName = "../Data/signals.dat";
    }

    no_of_signals = get_num_signals(signalsFileName);
    if(!no_of_signals)
    {
        cerr << "SignalAnalyzer Error: SignalAnalyzerList class." << endl
             << "Signal_Analyzer_List(const char*) method" << endl
             << "No signals found on file: "<< signalsFileName  << endl;

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

Signal_Analyzer_List::Signal_Analyzer_List(const char* signalsFileName, const vector<unsigned int>& signalIDList)
{
    //--Remove old output files--//
    system("exec rm -r ../Output/*");

    if(!signalsFileName)
    {
        signalsFileName = "../Data/signals.dat";
    }

    no_of_signals = signalIDList.size();
    if(!no_of_signals)
    {
        cerr << "SignalAnalyzer Error: SignalAnalyzerList class." << endl
             << "Signal_Analyzer_List(const char*, , const vector<unsigned int>&) method" << endl
             << "No signals found on file: "<< signalsFileName  << endl;

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

    fstream inputFile;
    inputFile.open(signalsFileName, ios::in);

    if(!inputFile.is_open())
    {
        cerr << "SignalAnalyzer Error: SignalAnalyzerList class." << endl
             << "get_num_signals(const char* const) method" << endl
             << "Cannot open Parameter file: "<< signalsFileName  << endl;

        exit(1);
    }

    string line;
    double dNum;
    size_t found;

    //--Omitting lines containing '#'--//
    do
    {
        getline(inputFile, line);
        found = line.find("#");
    }while(found != string::npos);

    stringstream ssLine(line);
    ssLine >> dNum;

    //--Extracting the number of signals on file--//
    while(ssLine >> dNum)
    {
        num_signals++;
    }


    cout << endl << "Number of signals on file: " << num_signals << endl << endl;
    inputFile.close();

    return num_signals;
}


// void filter_signal(void) method

/// This method filters crests and troughs of all the signals in the list.
/// Each signal is filteres by taking as reference, all other signals in the list.
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

    //--Show the number of Crests and Troughs found, for each signal, after filtering--//
    for(unsigned int i=0; i<no_of_signals; i++)
    {
        cout << "Signal_" << S[i].get_signal_id() << ": No. of Crests found = " << S[i].get_crest_time_size()
             << "   No. of Trough found = " << S[i].get_trough_time_size() << endl;
    }
}


// void crop_signal_length(const double, const double) method

/// This method crops all signals in the list, by removing all crests and troughs outside the limit [lower_limit:upper_limit].
/// @param lower_limit Time value below which signals need to be cropped.
/// @param upper_limit Time value above which signals need to be cropped.

void Signal_Analyzer_List::crop_signal_length(const double lower_limit, const double upper_limit)
{
    if(no_of_signals == 0)
    {
        cerr << "SignalAnalyzer Error: SignalAnalyzerList class." << endl
             << "void crop_signal_length(const double, const double) method" << endl
             << "Signals List is empty" << endl;

        exit(1);
    }

    cout << endl << "Cropping signals beyond the range [" << lower_limit << ":" << upper_limit << "]" << endl;

    for(unsigned int i=0; i<no_of_signals; i++)
    {
        S[i].crop_length(lower_limit, upper_limit);
    }

    //--Show the number of Crests and Troughs, for each signal, after cropping--//
    for(unsigned int i=0; i<no_of_signals; i++)
    {
        cout << "Signal_" << S[i].get_signal_id() << ": No. of Crests found = " << S[i].get_crest_time_size()
             << "   No. of Trough found = " << S[i].get_trough_time_size() << endl;
    }
}


// void show_signal_amplitudes(void) method

/// Estimates and displays average amplitude of all the signals on the list.

void Signal_Analyzer_List::show_signal_amplitudes(void)
{
    cout << endl << "         Signal Amplitudes" << endl;
    for(unsigned int i=0; i<no_of_signals; i++)
    {
        cout << "Signal_" << S[i].get_signal_id() << ": " << S[i].estimate_amplitude() << endl;
    }
}


// void show_signal_offset(void) method

/// Estimates and displays average offset of all the signals on the list.

void Signal_Analyzer_List::show_signal_offsets(void)
{
    cout << endl << "         Signal Offsets" << endl;
    for(unsigned int i=0; i<no_of_signals; i++)
    {
        cout << "Signal_" << S[i].get_signal_id() << ": " << S[i].estimate_offset() << endl;
    }
}


// void calculate_phaseVector_crest(void) method

/// Esimates phase difference between all pairs of signals on the list, as a vector,
/// and then stores the same on a plottable file. Calculate the phase difference based
/// on signal crest values.

void Signal_Analyzer_List::calculate_phaseVector_crest(void)
{
    vector<vector<vector<double> > > phase;
    vector<double> time;

    fstream phaseFile_180;
    fstream phaseFile_360;

    //--Calculate phase difference between signals, for every pair of signals.
    for(unsigned int i=0; i<no_of_signals-1; i++)
    {
        for(unsigned int j=i+1; j<no_of_signals; j++)
        {
            phase.push_back(S[i].calculate_phase_crest(S[j]));
        }
    }

    phaseFile_180.open("../Output/phase180.dat", ios::out);
    phaseFile_360.open("../Output/phase360.dat", ios::out);

    //--Accumulate all unique time values from all the phase vectors.
    for(unsigned int i=0; i<phase.size(); i++)
    {
        for(unsigned int j=0; j<phase[i].size(); j++)
        {
            if(find(time.begin(), time.end(), phase[i][j][0])==time.end())
            {
                time.push_back(phase[i][j][0]);
            }
        }
    }

    //--Sort the time vector in accending order.
    sort(time.begin(), time.end());

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

        phaseFile_180 << endl;
        phaseFile_360 << endl;
    }

    phaseFile_180.close();
    phaseFile_360.close();
}


// void calculate_phaseVector_trough(void) method

/// Esimates phase difference between all pairs of signals on the list, as a vector,
/// and then stores the same on a plottable file. Calculate the phase difference based
/// on signal trough values.

void Signal_Analyzer_List::calculate_phaseVector_trough(void)
{
    vector<vector<vector<double> > > phase;
    vector<double> time;

    fstream phaseFile_180;
    fstream phaseFile_360;

    //--Calculate phase difference between signals, for every pair of signals.
    for(unsigned int i=0; i<no_of_signals-1; i++)
    {
        for(unsigned int j=i+1; j<no_of_signals; j++)
        {
            phase.push_back(S[i].calculate_phase_trough(S[j]));
        }
    }

    phaseFile_180.open("../Output/phase180.dat", ios::out);
    phaseFile_360.open("../Output/phase360.dat", ios::out);

    //--Accumulate all unique time values from all the phase vectors.
    for(unsigned int i=0; i<phase.size(); i++)
    {
        for(unsigned int j=0; j<phase[i].size(); j++)
        {
            if(find(time.begin(), time.end(), phase[i][j][0])==time.end())
            {
                time.push_back(phase[i][j][0]);
            }
        }
    }

    //--Sort the time vector in accending order.
    sort(time.begin(), time.end());

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

        phaseFile_180 << endl;
        phaseFile_360 << endl;
    }

    phaseFile_180.close();
    phaseFile_360.close();
}


// void show_signal_phase_relation_crest(void) method

/// Estimates and displays average phase difference between all pairs of signals on the list.
/// Calculate the phase difference based on signal crest values.

void Signal_Analyzer_List::show_phase_relation_crest(void)
{
    double avg_phase_diff;

    cout << endl << "         Crest based phase difference between pairs of signals (-180°, 180°]" << endl;

    //--Calculate phase difference between signals, for every pair of signals.
    for(unsigned int i=0; i<no_of_signals-1; i++)
    {
        for(unsigned int j=i+1; j<no_of_signals; j++)
        {
            vector<vector<double> > phaseVector;
            vector<double> phaseRelation;

            phaseVector = S[i].calculate_phase_crest(S[j]);

            for(unsigned int k=0; k<phaseVector.size(); k++)
            {
                phaseRelation.push_back(phaseVector[k][1]);
            }
            avg_phase_diff = accumulate(phaseRelation.begin(), phaseRelation.end(), 0.0)/phaseRelation.size();

            cout << "Signal_" << S[i].get_signal_id() << " AND Signal_"
                 << S[j].get_signal_id() << ": " << avg_phase_diff << "°" << endl;
        }
    }

}


// void show_signal_phase_relation_trough(void) method

/// Estimates and displays average phase difference between all pairs of signals on the list.
/// Calculate the phase difference based on signal trough values.

void Signal_Analyzer_List::show_phase_relation_trough(void)
{
    double avg_phase_diff;

    cout << endl << "         Trough based phase difference between pairs of signals (-180°, 180°]" << endl;

    //--Calculate phase difference between signals, for every pair of signals.
    for(unsigned int i=0; i<no_of_signals-1; i++)
    {
        for(unsigned int j=i+1; j<no_of_signals; j++)
        {
            vector<vector<double> > phaseVector;
            vector<double> phaseRelation;

            phaseVector = S[i].calculate_phase_trough(S[j]);

            for(unsigned int k=0; k<phaseVector.size(); k++)
            {
                phaseRelation.push_back(phaseVector[k][1]);
            }
            avg_phase_diff = accumulate(phaseRelation.begin(), phaseRelation.end(), 0.0)/phaseRelation.size();

            cout << "Signal_" << S[i].get_signal_id() << " AND Signal_"
                 << S[j].get_signal_id() << ": " << avg_phase_diff << "°" << endl;
        }
    }

}


// void show_signal_frequency(void) method

/// Estimates and displays average frequency of all the signals on the list.

void Signal_Analyzer_List::show_signal_frequency(void)
{
    vector<double> frequency;
    double avg_freq;

    for(unsigned int i=0; i<no_of_signals; i++)
    {
        frequency.push_back(S[i].estimate_frequency());
    }
    avg_freq = accumulate(frequency.begin(), frequency.end(), 0.0)/frequency.size();

    cout << endl << "Signal Frequency: " << avg_freq << endl;
}


// void show_signal_range(void) method

/// Estimates and displays oscillation range of all the signals on the list.

void Signal_Analyzer_List::show_signal_range(void)
{
    double amplitude;
    double offset;

    cout << endl << "         Signal Range" << endl;
    for(unsigned int i=0; i<no_of_signals; i++)
    {
        amplitude = S[i].estimate_amplitude();
        offset = S[i].estimate_offset();

        cout << "Signal_" << S[i].get_signal_id() << ": [" << (amplitude + offset) << ":" << (-amplitude + offset) << "]" << endl;
    }
}
