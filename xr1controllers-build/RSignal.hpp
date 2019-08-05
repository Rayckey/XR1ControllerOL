#ifndef RSIGNAL_HPP
#define RSIGNAL_HPP

#include <functional>
#include <map>
#include <vector>

template <typename... Args>
class RSignal
{
public:
    RSignal():m_nCurrentId(0){}
    RSignal(RSignal const & other):m_nCurrentId(0){}

    template <typename T>
    /**
     * @brief connect_member Signal to subscribe to
     * @param inst Pointer to mother object
     * @func reference to member function
     * @return Index of this member, can be used to remove this signal
     */
    int subscribeSignal(T * instance_of_object,void (T::*func)(Args...)){
        return s_Connect([=](Args... args){
            (instance_of_object->*func)(args...);
        });
    }

    template <typename T>
    /**
     * @brief connect_member Signal to subscribe to
     * @param inst Pointer to mother object
     * @func reference to member function
     * @return Index of this connection, can be used to remove this signal
     */
    int subscribeSignal(T * instance_of_object, void (T::*func)(Args...)const){
        return s_Connect([=](Args... args){
            (instance_of_object->*func)(args...);
        });
    }
    /**
     * @brief s_Connect Signal to subscribe to
     * @param slot reference to member slot
     * @return Index of this connection, can be used to remove this signal
     */
    int s_Connect(std::function<void(Args...)>const & slot)const{
        m_slots.insert(std::make_pair(++m_nCurrentId,slot));
        return m_nCurrentId;
    }
    /**
     * @brief s_Disconnect Unsubscribe Signal 
     * @param id The id given during subscription
     */
    void s_Disconnect(int id)const{
        m_slots.erase(id);
    }
    /**
     * @brief s_DisconnectAll What else did you want?
     */
    void s_DisconnectAll()const{
        m_slots.clear();
    }
    /**
     * @brief s_Emit Emit Signal
     * @param p Signal parameters
     */
    void s_Publish(Args... p){
        for (auto it : m_slots) {
            it.second(p...);
        }
    }
    /**
     * @brief s_Disconnect Disconnect a list of signals
     * @param idList List of signal IDs.
     */
    void s_Disconnect(const std::vector<int> idList)const{
        for(int i=0;i<idList.size();++i)
            s_Disconnect(idList[i]);
    }
//    RSignal & operator =(RSignal const & other){
//        s_DisconnectAll();
//        return *this;
//    }

private:
    mutable int m_nCurrentId;
    mutable std::map<int,std::function<void (Args...)>> m_slots;
};
#endif // CSIGNAL_HPP
