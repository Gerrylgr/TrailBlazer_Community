#ifndef LOCAL_PLANNER__MPPI__CORE__CRITIC_MANAGER_HPP_
#define LOCAL_PLANNER__MPPI__CORE__CRITIC_MANAGER_HPP_

#include "local_planner/MPPI/core/critic_function.hpp"

#include <memory>
#include <utility>
#include <vector>

namespace local_planner::mppi_core
{

    class CriticManager
    {
    public:
        void addCritic(std::unique_ptr<CriticFunction> critic)
        {
            if (critic)
            {
                critics_.push_back(std::move(critic));
            }
        }

        void score(CriticData & data) const
        {
            for (const auto & critic : critics_)
            {
                critic->score(data);
            }
        }

        std::size_t size() const
        {
            return critics_.size();
        }

    private:
        std::vector<std::unique_ptr<CriticFunction>> critics_;
    };

}  // namespace local_planner::mppi_core

#endif