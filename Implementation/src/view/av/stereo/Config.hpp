/**
 * @file config.h
 * @author  Raúl Quintero Mínguez <raul.quintero@aut.uah.es>, David Fernández LLorca <llorca@aut.uah.es>. ISIS Research Group. University of Alcalá.
 * @version 1.0
 *
 * @section DESCRIPTION
 *
 * Project: SmartCross
 * 
 * This file contains the headers functions to read the configuration file.
 * 
 */

#ifndef CONFIG_H
#define	CONFIG_H


#include <src/view/av/stereo/Structs.hpp>


namespace src
{
    namespace view
    {
       namespace av
       {
           namespace stereo
           {
               class Config
               {
                   public:
                       int ReadConfFile(Structs::t_ConfigFile *conf_file, Structs::t_Limits *limits, int *waiting_areas, int *crossing_areas);

                       char* getCmdOption(char ** begin, char ** end, const std::string & option);

                       bool cmdOptionExists(char** begin, char** end, const std::string& option);
               };
           }
       }
    }
}

#endif	/* CONFIG_H */

