#include "ModifiedTasks.h"

#include <mc_rtc/gui/NumberInput.h>
#include <mc_rtc/gui/Transform.h>

#include <mc_filter/utils/clamp.h>
#include <mc_rbdyn/ZMP.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/ConfigurationHelpers.h>
#include <mc_rtc/constants.h>
#include <mc_tasks/MetaTaskLoader.h>

#include <chrono>

namespace mc_tasks
{
    EndEffectorTask_NoGUI::EndEffectorTask_NoGUI(const std::string &bodyName, const mc_rbdyn::Robots &robots, unsigned int robotIndex,
                                                 double stiffness, double weight) : EndEffectorTask(robots.robot(robotIndex).frame(bodyName), stiffness, weight)
    {
    }

    void EndEffectorTask_NoGUI::addToGUI(mc_rtc::gui::StateBuilder &gui)
    {
        MetaTask::addToGUI(gui);
        gui.addElement({"Tasks", name_},
                       mc_rtc::gui::Transform("pos", [this]()
                                              { return frame().position(); }));
        gui.addElement({"Tasks", name_, "Gains", "Position"},
                       mc_rtc::gui::NumberInput(
                           "stiffness", [this]()
                           { return this->positionTask->stiffness(); },
                           [this](const double &s)
                           { this->positionTask->setGains(s, this->positionTask->damping()); }),
                       mc_rtc::gui::NumberInput(
                           "damping", [this]()
                           { return this->positionTask->damping(); },
                           [this](const double &d)
                           { this->positionTask->setGains(this->positionTask->stiffness(), d); }),
                       mc_rtc::gui::NumberInput(
                           "stiffness & damping", [this]()
                           { return this->positionTask->stiffness(); },
                           [this](const double &g)
                           { this->positionTask->stiffness(g); }),
                       mc_rtc::gui::NumberInput(
                           "weight", [this]()
                           { return this->positionTask->weight(); },
                           [this](const double &w)
                           { this->positionTask->weight(w); }));
        gui.addElement({"Tasks", name_, "Gains", "Orientation"},
                       mc_rtc::gui::NumberInput(
                           "stiffness", [this]()
                           { return this->orientationTask->stiffness(); }, [this](const double &s)
                           { this->orientationTask->setGains(s, this->orientationTask->damping()); }),
                       mc_rtc::gui::NumberInput(
                           "damping", [this]()
                           { return this->orientationTask->damping(); }, [this](const double &d)
                           { this->orientationTask->setGains(this->orientationTask->stiffness(), d); }),
                       mc_rtc::gui::NumberInput(
                           "stiffness & damping", [this]()
                           { return this->orientationTask->stiffness(); },
                           [this](const double &g)
                           { this->orientationTask->stiffness(g); }),
                       mc_rtc::gui::NumberInput(
                           "weight", [this]()
                           { return this->orientationTask->weight(); },
                           [this](const double &w)
                           { this->orientationTask->weight(w); }));
    }
} // namespace mc_tasks
