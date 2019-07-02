#ifndef CROSSTIMER_H
#define CROSSTIMER_H
#include <chrono>
#include <functional>
#include <thread>
#include <condition_variable>
#include <mutex>
#include <atomic>
#include <memory>
#include <chrono>
#include <unordered_map>
#include <set>

using namespace std;
class CrossTimer
{
public:
    using timer_id = uint64_t;
    static timer_id constexpr no_timer = timer_id(0);
    using handle_type = std::function<void()>;
    using Lock = std::mutex;
    using ScopeLock = std::unique_lock<Lock>;
    using Conditionval = std::condition_variable;
    using Clock = std::chrono::steady_clock;
    using Timestamp = std::chrono::time_point<Clock>;
    using Duration = std::chrono::milliseconds;

    template<typename ... Args>
    using bound_handle_type = std::function<void(Args ...)>;

    using millisec = int64_t;

    explicit CrossTimer();
    CrossTimer(CrossTimer && other);
    CrossTimer(timer_id id,Timestamp next,Duration period,handle_type handler);
    ~CrossTimer();
    void start(millisec msPeriod, handle_type handler);
    void singleShot(millisec msDelay, handle_type handler);
    void stop();
private:
    timer_id timerId;
    Timestamp next;
    Duration period;
    handle_type handler;
    std::unique_ptr<Conditionval> waitCond;
    bool isRunning;
private:
    class TimerThread
    {
    public:
        static TimerThread& global();
        TimerThread();
        ~TimerThread();
        timer_id addTimer(millisec msDelay,millisec msPeriod, handle_type handler);
        timer_id addTimer(CrossTimer * timer);
//        template<typename... Args>
//        timer_id addTimer(millisec msDelay, millisec msPeriod, bound_handle_type<Args...> handler,Args&& ...args);

        // See clear() to wipe out all timers in one go
        //
        bool clearTimer(timer_id id);

        // Destroy all timers, but preserve id uniqueness
        // This carefully makes sure every timer is not
        // executing its callback before destructing it
        void clear();

        // Peek at current state
        std::size_t size() const ;
        bool empty() const ;

    private:

        struct NextActiveCompator
        {
            bool operator()(CrossTimer& a,CrossTimer& b)const
            {
                return a.next < b.next;
            }
        };

        using QueueValue = std::reference_wrapper<CrossTimer>;
        using Queue = std::multiset<QueueValue,NextActiveCompator>;
        using TimerMap = std::unordered_map<timer_id,CrossTimer*>;

        void timerThreadWork();
        bool destroyImpl(ScopeLock & lock,TimerMap::iterator i, bool notify);
        timer_id nextId;
        TimerMap active;
        Queue queue;
        mutable Lock sync;
        Conditionval wakeUp;
        std::thread workerThread;
        bool done;
    };

};

#endif // CROSSTIMER_H
