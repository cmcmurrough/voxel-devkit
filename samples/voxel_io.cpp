/***********************************************************************************************************************
 * @file voxel_io.cpp
 * @brief Example for utilizing voxel IO
 *
 * Example program for accessing voxel non-camera interfaces (internal temperatures, indicator LEDs, RS-232, etc.)
 *
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/

#include <Voxel.h>
#include <signal.h>
#include <thread>
#include <chrono>

// function prototypes
void exitHandler(int terminationCode);

/*******************************************************************************************************************//**
* @brief gracefully exits the program after deallocating shared memory variables
* @param[in] code exit status code
* @author Christopher D. McMurrough
***********************************************************************************************************************/
void exitHandler(int terminationCode)
{
    // release the GPIO pins
    Voxel::releaseLEDs();

    // return termination code
    if(terminationCode == EXIT_FAILURE)
    {
        exit(EXIT_FAILURE);
    }
    else
    {
        exit(EXIT_SUCCESS);
    }
}

/***********************************************************************************************************************
 * @brief program entry point
 * @param[in] argc number of command line arguments
 * @param[in] argv string array of command line arguments
 * @returnS return code (0 for normal termination)
 * @author Christopher D. McMurrough
 **********************************************************************************************************************/
int main(int argc, char *argv[])
{
    // create the program termination event handler (ctrl+c)
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = exitHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);

    // setup the GPIO pins
    Voxel::initializeLEDs(true, true);

    // flash LEDs until user quite program with ctrl+c
    while(true)
    {
        std::cout << "Toggling LED states" << std::endl;
        Voxel::setRedLED(true);
        Voxel::setGreenLED(false);
        std::this_thread::sleep_for (std::chrono::milliseconds(200));

        std::cout << "Toggling LED states" << std::endl;
        Voxel::setRedLED(false);
        Voxel::setGreenLED(true);
        std::this_thread::sleep_for (std::chrono::milliseconds(200));

        // read processor temperatures
        std::vector<float> temperatures;
        Voxel::getTemperatures(temperatures);
        for(int i = 0; i < temperatures.size(); i++)
        {
            std::cout << "   zone " << i << " temperature: " << temperatures.at(i) << std::endl;
        }
    }

    // exit program
    exitHandler(0);
}
