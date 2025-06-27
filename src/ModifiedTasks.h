#pragma once

#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/lipm_stabilizer/StabilizerTask.h>

namespace mc_tasks
{
    struct MC_TASKS_DLLAPI EndEffectorTask_NoGUI : public EndEffectorTask
    {
    public:
        EndEffectorTask_NoGUI(const std::string &bodyName,
                              const mc_rbdyn::Robots &robots,
                              unsigned int robotIndex,
                              double stiffness = 10.0,
                              double weight = 1000.0);

    protected:
        void addToGUI(mc_rtc::gui::StateBuilder &gui) override;
    };

}