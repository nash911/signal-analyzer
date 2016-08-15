/********************************************************************************************/
/*                                                                                          */
/*   SignalAnalyzer: A project for extracting signal parameters from raw signal data.       */
/*                                                                                          */
/*   S I G N A L   A N A L Y Z E R   C L A S S                                              */
/*                                                                                          */
/*   Avinash Ranganath                                                                      */
/*   Robotics Lab, Department of Systems Engineering and Automation                         */
/*   University Carlos III of Mardid(UC3M)                                                  */
/*   Madrid, Spain                                                                          */
/*   E-mail: nash911@gmail.com                                                              */
/*   http://roboticslab.uc3m.es/roboticslab/persona.php?id_pers=104                         */
/*                                                                                          */
/********************************************************************************************/

#include "signal_analyzer.h"

// CONSTRUCTOR

/// Creates a Signal Analyzer object, initializing the signal ID.
/// Extracts the signal data fom the file, whose path and name is passed as a parameter.
/// Calculates the signal mean.
/// Selects the crests and troughs from the signal data.
/// @param fileName Path and name of the file containing the signal data.
/// @param sig_id Id of the signal.

Signal_Analyzer::Signal_Analyzer(const char* const fileName, const unsigned int sig_id)
{
    signal_id = sig_id;

    extract_signal_data_from_file(fileName);

    //--Calculate signal mean--//
    double sum = accumulate(signal.begin(), signal.end(), 0.0);
    signal_mean = sum / signal.size();

    select_signal_crest();
    select_signal_trough();
}


// void extract_signal_data_from_file(const char* const) method

/// This method extracts the signal data from the file, the location of which is passed as parameter.
/// @param fileName Path and name of the file containing the signal data.

void Signal_Analyzer::extract_signal_data_from_file(const char* const fileName)
{
    fstream inputFile;

    inputFile.open(fileName, ios::in);
    if(!inputFile.is_open())
    {
        cerr << "SignalAnalyzer Error: SignalAnalyzer class." << endl
             << "extract_signal_data_from_file(const char* const) method." << endl
             << "Cannot open Parameter file: "<< fileName  << endl;

        exit(1);
    }

    string line;
    double dNum;
    size_t found;

    //--Extracting the time and signal data from the file--//
    do
    {
        //--Omitting lines containing '#'--//
        do
        {
            getline(inputFile, line);
            found = line.find("#");
        }while(found != string::npos);

        stringstream ssLine;
        ssLine << line;

        ssLine >> dNum;
        time.push_back(dNum);

        for(unsigned int i=0; i<signal_id; i++)
        {
            ssLine >> dNum;
        }
        signal.push_back(dNum);

    }while(!inputFile.eof());
    inputFile.close();
}


// void crop_length(const double, const double)

/// This method crops a signal by removing all crests and troughs outside the limit [lower_limit:upper_limit].
/// @param lower_limit Time value below which the signal needs to be cropped.
/// @param upper_limit Time value above which the signal needs to be cropped.

void Signal_Analyzer::crop_length(const double lower_limit, const double upper_limit)
{
    bool crest_updated = false;
    bool trough_updated = false;

    //cout << endl << "Signal_" << signal_id << endl;

    //--Remove crests with t < lower_limit--//
    while(signal_crest_time.front() < lower_limit)
    {
        crest_updated = true;
        //cout << " Removed Crest: " << signal_crest.front() << "  @ t: " << signal_crest_time.front() << endl;

        signal_crest.erase(signal_crest.begin());
        signal_crest_time.erase(signal_crest_time.begin());
    }

    //--Remove crests with t > upper_limit--//
    while(signal_crest_time.back() > upper_limit)
    {
        crest_updated = true;
        //cout << " Removed Crest: " << signal_crest.back() << "  @ t: " << signal_crest_time.back() << endl;

        signal_crest.pop_back();
        signal_crest_time.pop_back();
    }

    //--Remove troughs with t < lower_limit--//
    while(signal_trough_time.front() < lower_limit)
    {
        trough_updated = true;
        //cout << " Removed Trough: " << signal_trough.front() << "  @ t: " << signal_trough_time.front() << endl;

        signal_trough.erase(signal_trough.begin());
        signal_trough_time.erase(signal_trough_time.begin());
    }

    //--Remove troughs with t > upper_limit--//
    while(signal_trough_time.back() > upper_limit)
    {
        trough_updated = true;
        //cout << " Removed Trough: " << signal_trough.back() << "  @ t: " << signal_trough_time.back() << endl;

        signal_trough.pop_back();
        signal_trough_time.pop_back();
    }

    //cout << endl;

    if(crest_updated)
    {
        //--Save updated crest data in file.
        fstream outputFile;
        stringstream ss;

        ss << "../Output/S" << signal_id << "_filtered_crest.dat";
        remove(ss.str().c_str());
        outputFile.open(ss.str().c_str(), ios::out);

        for(unsigned int i=0; i<signal_crest.size(); i++)
        {
            outputFile << signal_crest_time[i] << " " << signal_crest[i] << endl;
        }
        outputFile.close();
    }

    if(trough_updated)
    {
        //--Save filtered trough data in file.
        fstream outputFile;
        stringstream ss;

        ss << "../Output/S" << signal_id << "_filtered_trough.dat";
        remove(ss.str().c_str());
        outputFile.open(ss.str().c_str(), ios::out);

        for(unsigned int i=0; i<signal_trough.size(); i++)
        {
            outputFile << signal_trough_time[i] << " " << signal_trough[i] << endl;
        }
        outputFile.close();
    }
}


// bool is_previous_signal_lower(const unsigned int) method

/// This method compares two consecutive points of a time varied signal.
/// Returns true if the previous signal point is lower.
/// Returns false if the previous signal point is higher.
/// Recursively calls itself if the two signal points are equal or if the previous signal point is detected to be noisy.
/// @param index Index of a point on a time varied signal.

bool Signal_Analyzer::is_previous_signal_lower(const unsigned int index) const
{
    if(index != 1)
    {
        if(signal[index] == signal[index-1])
        {
            return(is_previous_signal_lower(index-1));
        }
        else if(signal[index] > signal[index-1])
        {
            if(fabs(signal[index]-signal[index-1]) < NOISE_THRESHOLD)
            {
                return(true);
            }
            else
            {
                //--If noise data detected, then continue searching by stepping over the noise data point--//
                return(is_previous_signal_lower(index-2));
            }
        }
        else if(signal[index] < signal[index-1])
        {
            return(false);
        }
    }
    else
    {
        if(signal[index] > signal[index-1])
        {
            return(true);
        }
        else if(signal[index] <= signal[index-1])
        {
            return(false);
        }
    }

    return false;
}


// bool is_previous_signal_higher(const unsigned int) method

/// This method compares two consecutive points of a time varied signal.
/// Returns true if the previous signal point is higher.
/// Returns false if the previous signal point is lower.
/// Recursively calls itself if the two signal points are equal or if the previous signal point is detected to be noisy.
/// @param index Index of a point on a time varied signal.

bool Signal_Analyzer::is_previous_signal_higher(const unsigned int index) const
{
    if(index != 1)
    {
        if(signal[index] == signal[index-1])
        {
            return(is_previous_signal_higher(index-1));
        }
        else if(signal[index] < signal[index-1])
        {
            if(fabs(signal[index]-signal[index-1]) < NOISE_THRESHOLD)
            {
                return(true);
            }
            else
            {
                //--If noise data detected, then continue searching by stepping over the noise data point--//
                return(is_previous_signal_higher(index-2));
            }
        }
        else if(signal[index] > signal[index-1])
        {
            return(false);
        }
    }
    else
    {
        if(signal[index] < signal[index-1])
        {
            return(true);
        }
        else if(signal[index] >= signal[index-1])
        {
            return(false);
        }
    }

    return false;
}


// void select_signal_crest(void) method

/// This method searches through the signal data and selets all potential signal points that could be crests of the signal.

void Signal_Analyzer::select_signal_crest(void)
{
    for(unsigned int i=1; i<time.size()-1; i++)
    {
        //--Naive noise filter
        if(signal[i] > signal_mean && (fabs(signal[i]-signal[i-1])<=NOISE_THRESHOLD && fabs(signal[i]-signal[i+1])<=NOISE_THRESHOLD))
        {
            if(signal[i] > signal[i+1])
            {
                if(is_previous_signal_lower(i))
                {
                    signal_crest.push_back(signal[i]);
                    signal_crest_time.push_back(time[i]);
                }
            }
        }
    }

    //--Save selected crest data in file--//
    fstream outputFile;
    stringstream ss;

    ss << "../Output/S" << signal_id << "_crest.dat";
    outputFile.open(ss.str().c_str(), ios::out);

    for(unsigned int i=0; i<signal_crest.size(); i++)
    {
        outputFile << signal_crest_time[i] << " " << signal_crest[i] << endl;
    }
    outputFile.close();
}


// void select_signal_trough(void) method

/// This method searches through the signal data and selets all potential signal points that could be troughs of the signal.

void Signal_Analyzer::select_signal_trough(void)
{
    for(unsigned int i=1; i<time.size()-1; i++)
    {
        //--Naive noise filter
        if(signal[i] < signal_mean && (fabs(signal[i]-signal[i-1])<=NOISE_THRESHOLD && fabs(signal[i]-signal[i+1])<=NOISE_THRESHOLD))
        {
            if(signal[i] < signal[i+1])
            {
                if(is_previous_signal_higher(i))
                {
                    signal_trough.push_back(signal[i]);
                    signal_trough_time.push_back(time[i]);
                }
            }
        }
    }

    //--Save selected trough data in file--//
    fstream outputFile;
    stringstream ss;

    ss << "../Output/S" << signal_id << "_trough.dat";
    outputFile.open(ss.str().c_str(), ios::out);

    for(unsigned int i=0; i<signal_trough.size(); i++)
    {
        outputFile << signal_trough_time[i] << " " << signal_trough[i] << endl;
    }
    outputFile.close();
}


// void filter_signal_crest(const Signal_Analyzer&) method

/// This method filters the crests vector of the signal.
/// It does so by retaining the highest crest that lies between two concurrent crests of the reference signal.
/// @param ref_sig A SignalAnalyzer object used as a reference to filter the crests of the signal.

void Signal_Analyzer::filter_signal_crest(const Signal_Analyzer& ref_sig)
{
    double temp_time;
    double temp_index;

    //--Weeding out noisy signal crest by selecting the highest signal crest between two reference signal ref_sig's crests--//
    for(unsigned int i=0; i<ref_sig.get_crest_time_size()-1; i++)
    {
        vector<double> multiple_crests_time;
        vector<unsigned int> crest_index;

        //--Collect all the signal crests between two concurrent ref_sig crests--//
        for(unsigned int j=0; j<signal_crest_time.size(); j++)
        {
            if(ref_sig.get_crest_time(i) <= signal_crest_time[j] && signal_crest_time[j] <= ref_sig.get_crest_time(i+1))
            {
                multiple_crests_time.push_back(signal_crest_time[j]);
                crest_index.push_back(j);
            }
            else if(signal_crest_time[j] > ref_sig.get_crest_time(i+1))
            {
                break;
            }
        }

        //--If there exists more than one signal crest between the current pair of adjacent ref_sig crests,
        //--then sort them in assending order, based on the height of the signal crest--//
        if(multiple_crests_time.size()>1)
        {
            for(unsigned int k=0; k<crest_index.size()-1; k++)
            {
                if(signal_crest[crest_index[k]] >= signal_crest[crest_index[k+1]])
                {
                    temp_time = multiple_crests_time[k];
                    multiple_crests_time[k] = multiple_crests_time[k+1];
                    multiple_crests_time[k+1] = temp_time;

                    temp_index = crest_index[k];
                    crest_index[k] = crest_index[k+1];
                    crest_index[k+1] = temp_index;
                }
            }

            //--Delete all but the highest signal crest that exist between the current pair of adjacent ref_sig crests--//
            for(unsigned int m=0; m<crest_index.size()-1; m++)
            {
                for(unsigned int n=0; n<signal_crest_time.size(); n++)
                {
                    if(signal_crest_time[n] == multiple_crests_time[m])
                    {
                        //--Delete both the non-highest signal crest and it's time, in the respective vectors--//
                        signal_crest.erase(signal_crest.begin() + n);
                        signal_crest_time.erase(signal_crest_time.begin() + n);

                        if(signal_crest.size() != signal_crest_time.size())
                        {
                            cout << endl << endl << " ERROR: SIZE OF signal_crest and signal_crest_time DO NOT MATCH." << endl;
                            exit(0);
                        }
                        break;
                    }
                }
            }
        }
    }

    //--Save filtered crest data in file.
    fstream outputFile;
    stringstream ss;

    ss << "../Output/S" << signal_id << "_filtered_crest.dat";
    outputFile.open(ss.str().c_str(), ios::out);

    for(unsigned int i=0; i<signal_crest.size(); i++)
    {
        outputFile << signal_crest_time[i] << " " << signal_crest[i] << endl;
    }
    outputFile.close();
}


// void filter_signal_trough(const Signal_Analyzer&) method

/// This method filters the troughs vector of the signal.
/// It does so by retaining the lowest trough that lies between two concurrent troughs of the reference signal.
/// @param ref_sig A SignalAnalyzer object used as a reference to filter the troughs of the signal.

void Signal_Analyzer::filter_signal_trough(const Signal_Analyzer& ref_sig)
{
    double temp_time;
    double temp_index;

    //--Weeding out noisy signal troughs, by selecting the highest signal trough between two reference signal ref_sig's troughs--//
    for(unsigned int i=0; i<ref_sig.get_trough_time_size()-1; i++)
    {
        vector<double> multiple_troughs_time;
        vector<unsigned int> trough_index;

        //--Collect all the signal troughs between two concurrent ref_sig troughs--//
        for(unsigned int j=0; j<signal_trough_time.size(); j++)
        {
            if(ref_sig.get_trough_time(i) <= signal_trough_time[j] && signal_trough_time[j] <= ref_sig.get_trough_time(i+1))
            {
                multiple_troughs_time.push_back(signal_trough_time[j]);
                trough_index.push_back(j);
            }
            else if(signal_trough_time[j] > ref_sig.get_trough_time(i+1))
            {
                break;
            }
        }

        //--If there exists more than one signal trough between the current pair of adjacent ref_sig troughs,...
        //--then sort them in descending order, based on the height of the signal trough--//
        if(multiple_troughs_time.size()>1)
        {
            for(unsigned int k=0; k<trough_index.size()-1; k++)
            {
                if(signal_trough[trough_index[k]] <= signal_trough[trough_index[k+1]])
                {
                    temp_time = multiple_troughs_time[k];
                    multiple_troughs_time[k] = multiple_troughs_time[k+1];
                    multiple_troughs_time[k+1] = temp_time;

                    temp_index = trough_index[k];
                    trough_index[k] = trough_index[k+1];
                    trough_index[k+1] = temp_index;
                }
            }

            //--Delete all but the highest signal trough that exist between the current pair of adjacent ref_sig troughs--//
            for(unsigned int m=0; m<trough_index.size()-1; m++)
            {
                for(unsigned int n=0; n<signal_trough_time.size(); n++)
                {
                    if(signal_trough_time[n] == multiple_troughs_time[m])
                    {
                        //--Delete both the non-highest signal trough and it's time, in the respective vectors--//
                        signal_trough.erase(signal_trough.begin() + n);
                        signal_trough_time.erase(signal_trough_time.begin() + n);

                        if(signal_trough.size() != signal_trough_time.size())
                        {
                            cout << endl << endl << " ERROR: SIZE OF signal_trough and signal_trough_time DO NOT MATCH." << endl;
                            exit(0);
                        }
                        break;
                    }
                }
            }
        }
    }

    //--Save filtered trough data in file.
    fstream outputFile;
    stringstream ss;

    ss << "../Output/S" << signal_id << "_filtered_trough.dat";
    outputFile.open(ss.str().c_str(), ios::out);

    for(unsigned int i=0; i<signal_trough.size(); i++)
    {
        outputFile << signal_trough_time[i] << " " << signal_trough[i] << endl;
    }
    outputFile.close();
}


// double estimate_amplitude(void) method

/// This method returns an estimate of the average amplitude of the signal.
/// μ_amplitude = μ_crest - (μ_crest + μ_trough)/2

double Signal_Analyzer::estimate_amplitude(void) const
{
    double amplitude_avg;
    double crest_avg;
    double trough_avg;
    double offset;

    crest_avg = accumulate(signal_crest.begin(), signal_crest.end(), 0.0)/signal_crest.size();
    trough_avg = accumulate(signal_trough.begin(), signal_trough.end(), 0.0)/signal_trough.size();

    offset = (crest_avg+trough_avg)/2.0;
    amplitude_avg = crest_avg - offset;

    return amplitude_avg;
}


// double estimate_offset(void) method

/// This method returns an estimate of the average offset of the signal.
/// μ_offset = (μ_crest + μ_trough)/2

double Signal_Analyzer::estimate_offset(void) const
{
    double offset_avg;
    double crest_avg;
    double trough_avg;

    crest_avg = accumulate(signal_crest.begin(), signal_crest.end(), 0.0)/signal_crest.size();
    trough_avg = accumulate(signal_trough.begin(), signal_trough.end(), 0.0)/signal_trough.size();

    offset_avg = (crest_avg+trough_avg)/2.0;

    return offset_avg;
}


// double estimate_frequency(void) method

/// This method returns an estimate of the average frequency of the signal.
/// It does so by first estimating the period between every concurrent pair of crests and trough.
/// Frequency = 1.0/Period
/// Finally returns average of the calculated frequencies.

double Signal_Analyzer::estimate_frequency(void) const
{
    vector<double> freqVec;
    double period;
    double freq_avg;

    //--Estimate frequenct based on crests--//
    for(unsigned int i=1; i<signal_crest_time.size(); i++)
    {
        //--frequency = 1.0/period--//
        period = signal_crest_time[i] - signal_crest_time[i-1];
        freqVec.push_back(1.0/period);
    }

    //--Estimate frequenct based on trough--//
    for(unsigned int i=1; i<signal_trough_time.size(); i++)
    {
        //--frequency = 1.0/period--//
        period = signal_trough_time[i] - signal_trough_time[i-1];
        freqVec.push_back(1.0/period);
    }

    freq_avg = accumulate(freqVec.begin(), freqVec.end(), 0.0)/freqVec.size();
    return (freq_avg);
}


// vector<vector<double> > calculate_phase_crest(const Signal_Analyzer&) method

/// This method returns the phase difference between the signal and the reference signal, calculated based on signal crests.
/// The format is a nx3 Matrix (2D Vector).
/// <ul>
/// <li> Time.
/// <li> Phase difference in the range of [0°,360°).
/// <li> Phase difference in the range of (-180°,180°].
/// </ul>
/// @param ref_sig A SignalAnalyzer object used as reference to calculate the signal's phase W.R.T the reference signal.

vector<vector<double> > Signal_Analyzer::calculate_phase_crest(const Signal_Analyzer& ref_sig) const
{
    vector<vector<double> > phase;
    vector<double> phase_individual(3);

    double period_start_time;
    double period_end_time;

    double phase_360;
    double phase_180;

    bool phase_flag;

    //--Calculating phase based on Crest values--//
    for(unsigned int i=0; i<signal_crest_time.size(); i++)
    {
        phase_flag = false;

        //--Search for the two reference signal ref_sig's crests, sandwitching the current signal crest--//
        for(unsigned int j=1; j<ref_sig.get_crest_time_size(); j++)
        {
            if(ref_sig.get_crest_time(j-1) <= signal_crest_time[i] && signal_crest_time[i] < ref_sig.get_crest_time(j))
            {
                period_start_time = ref_sig.get_crest_time(j-1);
                period_end_time = ref_sig.get_crest_time(j);

                phase_flag = true;
                break;
            }
        }

        if(phase_flag)
        {
            phase_360 = (signal_crest_time[i]-period_start_time)/(period_end_time-period_start_time) * 360.0;

            if(phase_360 > 180.0)
            {
                phase_180 = phase_360-360.0;
            }
            else
            {
                phase_180 = phase_360;
            }

            phase_individual[0] = signal_crest_time[i];
            phase_individual[1] = phase_180;
            phase_individual[2] = phase_360;

            phase.push_back(phase_individual);
        }
    }

    return phase;
}


// vector<vector<double> > calculate_phase_trough(const Signal_Analyzer&) method

/// This method returns the phase difference between the signal and the reference signal, calculated based on signal troughs.
/// The format is a nx3 Matrix (2D Vector).
/// <ul>
/// <li> Time.
/// <li> Phase difference in the range of [0°,360°).
/// <li> Phase difference in the range of (-180°,180°].
/// </ul>
/// @param ref_sig A SignalAnalyzer object used as reference to calculate the signal's phase W.R.T the reference signal.

vector<vector<double> > Signal_Analyzer::calculate_phase_trough(const Signal_Analyzer& ref_sig) const
{
    vector<vector<double> > phase;
    vector<double> phase_individual(3);

    double period_start_time;
    double period_end_time;

    double phase_360;
    double phase_180;

    bool phase_flag;

    //--Calculating phase based on Trough values--//
    for(unsigned int i=0; i<signal_trough_time.size(); i++)
    {
        phase_flag = false;

        //--Search for the two reference signal ref_sig's troughs, sandwitching the current signal trough--//
        for(unsigned int j=1; j<ref_sig.get_trough_time_size(); j++)
        {
            if(ref_sig.get_trough_time(j-1) <= signal_trough_time[i] && signal_trough_time[i] < ref_sig.get_trough_time(j))
            {
                period_start_time = ref_sig.get_trough_time(j-1);
                period_end_time = ref_sig.get_trough_time(j);

                phase_flag = true;
                break;
            }
        }

        if(phase_flag)
        {
            phase_360 = (signal_trough_time[i]-period_start_time)/(period_end_time-period_start_time) * 360.0;

            if(phase_360 > 180.0)
            {
                phase_180 = phase_360-360.0;
            }
            else
            {
                phase_180 = phase_360;
            }

            phase_individual[0] = signal_trough_time[i];
            phase_individual[1] = phase_180;
            phase_individual[2] = phase_360;

            phase.push_back(phase_individual);
        }
    }

    return phase;
}


// double get_crest_time(const unsigned int) method

/// This method returns a single value of the crest time vector.
/// @param i Index of crest time vector.

double Signal_Analyzer::get_crest_time(const unsigned int i) const
{
    if(i >= signal_crest_time.size())
    {
        cerr << "SignalAnalyzer Error: SignalAnalyzer class." << endl
             << "double get_crest_time(const unsigned int) method" << endl
             << i << " should be < signal_crest_time.size(): "<< signal_crest_time.size()  << endl;

        exit(1);
    }
    else
    {
        return signal_crest_time[i];
    }
}


// double get_trough_time(const unsigned int) method

/// This method returns a single value of the trough time vector.
/// @param i Index of trough time vector.

double Signal_Analyzer::get_trough_time(const unsigned int i) const
{
    if(i >= signal_trough_time.size())
    {
        cerr << "SignalAnalyzer Error: SignalAnalyzer class." << endl
             << "double get_trough_time(const unsigned int) method" << endl
             << i << " should be < signal_trough_time.size(): "<< signal_trough_time.size()  << endl;

        exit(1);
    }
    else
    {
        return signal_trough_time[i];
    }
}


// unsigned int get_crest_time_size(void) method

/// This method returns the size of the crest time vector.

unsigned int Signal_Analyzer::get_crest_time_size(void) const
{
    return signal_crest_time.size();
}


// unsigned int get_trough_time_size(void) method

/// This method returns the size of the trough time vector.

unsigned int Signal_Analyzer::get_trough_time_size(void) const
{
    return signal_trough_time.size();
}


// unsigned int get_signal_id(void) method

/// This method returns the ID of the signal.

unsigned int Signal_Analyzer::get_signal_id(void) const
{
    return signal_id;
}
