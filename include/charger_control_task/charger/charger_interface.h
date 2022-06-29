#ifndef CHARGER
#define CHARGER

#include <string>

namespace iCube
{
    class Charger;
}

class Charger
{
    public:

    enum class Status
    {
        CHARGING,
        NOT_CHARGING,
        UNKNOWN,
        ERROR
    };

    /**
     * @brief Return string name of enum
     * 
     * @param status 
     * @return std::string 
     */
    std::string status_to_string(Status status)
    {
        switch(status)
        {
            case Status::CHARGING: return "Charging";
            case Status::NOT_CHARGING: return "Not Charging";
            case Status::UNKNOWN: return "Unknown";
            case Status::ERROR: return "Error";
            default: return "Invalid State";
        }
    }

    virtual bool enable_charger(void) = 0;
    virtual bool disable_charger(void) = 0;

    /**
     * @brief Cancel function need only be implemented for chargers 
     * which take a significant time duration for enabling/disabling
     * 
     */
    virtual void cancel(void) {};

    virtual Charger::Status get_status(void) = 0;
};

#endif // CHARGER
