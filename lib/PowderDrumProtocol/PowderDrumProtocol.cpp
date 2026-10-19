/*
Title: PowderDrumProtocol.cpp
Purpose:
- Implements access to generated protocol metadata.
*/

#include "PowderDrumProtocol.h"

namespace PowderDrumProtocol
{

///////////////////////////////////////////////////////////////////////////////////////////////////

    bool findParameter(uint8_t parameterID, ParamMeta& metadata)
    {
        for (uint8_t i = 0; i < PARAM_TABLE_LEN; i++)
        {
            ParamMeta entry;
            memcpy_P(&entry, &PARAM_TABLE[i], sizeof(ParamMeta));

            if (entry.id == parameterID)
            {
                metadata = entry;
                return true;
            }
        }

        return false;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////

    bool exists(uint8_t parameterID)
    {
        ParamMeta metadata;
        return findParameter(parameterID, metadata);
    }

///////////////////////////////////////////////////////////////////////////////////////////////////

    bool isReadable(uint8_t parameterID)
    {
        ParamMeta metadata;
        if (!findParameter(parameterID, metadata)) return false;

        return metadata.access & PARAM_ACCESS_R;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////

    bool isWritable(uint8_t parameterID)
    {
        ParamMeta metadata;
        if (!findParameter(parameterID, metadata)) return false;

        return metadata.access & PARAM_ACCESS_W;
    }

///////////////////////////////////////////////////////////////////////////////////////////////////

    ParamType type(uint8_t parameterID)
    {
        ParamMeta metadata;
        if (!findParameter(parameterID, metadata)) return PARAM_TYPE_FLOAT;

        return static_cast<ParamType>(metadata.type);
    }

///////////////////////////////////////////////////////////////////////////////////////////////////

}