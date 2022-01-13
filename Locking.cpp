#include <shared_mutex>

namespace detail {

// traits:

template<class Mutex>
struct WriteLock
{
    using Type = std::unique_lock<Mutex>;
};

template<class Mutex>
using WriteLockType = typename WriteLock<Mutex>::Type;


template<class Mutex>
struct ReadLock
{
    using Type = std::unique_lock<Mutex>;
};

template<>
struct ReadLock<std::shared_timed_mutex>
{
    using Type = std::shared_lock<std::shared_timed_mutex>;
};

template<>
struct ReadLock<std::shared_mutex>
{
    using Type = std::shared_lock<std::shared_mutex>;
};

template<class Mutex>
using ReadLockType = typename ReadLock<Mutex>::Type;


template<typename Type, class Mutex, class ReadLock, class WriteLock, class>
class LockingSFINAE;

template<typename T>
struct isLockingIterateHelper : std::false_type {};

template <typename... Args>
struct isLockingIterateHelper<LockingSFINAE<Args...>> : std::true_type {};

template <typename Key, typename... Args>
struct isLockingIterateHelper<std::pair<const Key, LockingSFINAE<Args...>>> : std::true_type {};

template<typename T>
struct isLockingIterate : public isLockingIterateHelper<std::decay_t<T>> {};

template<typename T>
constexpr bool isLockingIterateValue = isLockingIterate<T>::value;


// access locked element:

template<typename Type, class Lock>
class Locked;

template<typename Type, class Lock>
class DereferencedForAssignment
{
public:
    
    Type operator=(Type element)
    {
        m_locked.m_element = std::move(element);
        return m_locked.m_element;
    }
    
private:
    
    Locked<Type, Lock> m_locked;
    
    DereferencedForAssignment(Locked<Type, Lock> locked) : m_locked{std::move(locked)}
    {}   
    
    friend Locked<Type, Lock>;
};


// passthrough to element holding locked mutex:

template<typename Type, class Lock>
class Locked
{
    friend DereferencedForAssignment<Type, Lock>;
    
    template<typename OtherType, class Mutex, class ReadLock, class WriteLock, class>
    friend class LockingSFINAE;
    
public:
    
    Type * operator->()
    {
        return &m_element;
    }
    
    // Only allow dereferencing as lvalue since the lock is released on destruction:
    Type & operator*() &
    {
        return m_element;
    }
    
    // Allow assigning when dereferencing as rvalue; then hold the lock:
    DereferencedForAssignment<Type, Lock> operator*() &&
    {
        return {std::move(*this)};
    }
    
    Locked() = delete;
    Locked(Locked & other) = delete;
    Locked(const Locked & other) = delete;
    Locked(volatile Locked & other) = delete;    
    Locked(const volatile Locked & other) = delete;
    
    Locked(Locked && other) = default;
    ~Locked() = default;
    
private:
    
    Type & m_element;
    
    Lock m_lock;
    
    template<typename Mutex>
    Locked(Type & element, Mutex & mutex) : 
        m_element{element},
        m_lock{mutex}
    {}
};



// container for element together with its mutex:

template<typename Type, class Mutex, class ReadLock, class WriteLock, typename = void>
class LockingSFINAE
{
    using ReadLocked = Locked<const Type, ReadLock>;
    
    using WriteLocked = Locked<Type, WriteLock>;
    
    template<typename OtherType, class Lock>
    friend class Locked;
    
public:
    
    template<typename...Args>
    LockingSFINAE(Args...args) : 
        m_element{args...},
        m_mutex{std::make_unique<Mutex>()}
    {}
        
    ReadLocked getReadLocked() const
    {
        return {m_element, *m_mutex};
    }
  
    WriteLocked getWriteLocked()
    {
        return {m_element, *m_mutex};
    }
    
protected:
    
    Type m_element;
    
    std::unique_ptr<Mutex> m_mutex;
};


template<class Type>
using IterateType = decltype(*std::cbegin(std::declval<Type>()));


template<typename Type, class Mutex, class ReadLock, class WriteLock>
class LockingSFINAE<Type, Mutex, ReadLock, WriteLock, std::void_t<IterateType<Type>>> :
    public LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int /* != void: use generic template. */>
{
    using Iterate = IterateType<Type>;
    
    using IterateLock = std::conditional_t<isLockingIterateValue<Iterate>, ReadLock, WriteLock>;
    
    using IterateLocked = Locked<Type, IterateLock>;
    
public:
    
    template<typename...Args>
    LockingSFINAE(Args...args) : LockingSFINAE<Type, Mutex, ReadLock, WriteLock, int>(args...)
    {}
    
    IterateLocked getIterateLocked()
    {
        return {this->m_element, *this->m_mutex};
    }
    
};


} // namespace detail


template<typename Type, 
    class Mutex = std::shared_timed_mutex, 
    class ReadLock = typename detail::ReadLockType<Mutex>, 
    class WriteLock = typename detail::WriteLockType<Mutex>>
using Locking = detail::LockingSFINAE<Type, Mutex, ReadLock, WriteLock, void>;



#include <iostream> 
#include <thread>
#include <vector>
#include <map>
#include <chrono>


template<typename Type>
class Dummy
{
public:
    
    template<typename...Args>
    Dummy(Args...args) : 
        m_element{args...}
    {}
        
    const Type * getReadLocked() const
    {
        return &m_element;
    }
    
    Type * getIterateLocked()
    {
        return &m_element;
    }
    
    Type * getWriteLocked()
    {
        return &m_element;
    }
    
private:
    
    Type m_element;
    
};


//TODO adopt work loads: add/remove some elements, update few/many elements, read few/many elements

template<class T>
int read(const T & container)
{
    int sum{};
    for (int unused=0; unused < 20; ++unused)
    {
        auto lockedContainer = container.getReadLocked();
        for (auto && element : *lockedContainer)
        {
            volatile int x = 0;
            for (int i=0; i<200; ++i)
            {
                x += i;
            }
            auto lockedElement = element.getReadLocked();
            sum += *lockedElement + x;
        }
    }
    return sum;
}


template<class T>
void write(T & container)
{  
    for (int unused=0; unused < 4; ++unused)
    {
        auto lockedContainer = container.getWriteLocked();
        for (auto && element : *lockedContainer)
        {
            volatile int x = 0;
            for (int i=0; i<500; ++i)
            {
                x += i;
            }
            auto lockedElement = element.getWriteLocked();
            *lockedElement = x;
        }
    }
} 


template<class T>
void update(T & container)
{  
    for (int unused=0; unused < 4; ++unused)
    {
        auto lockedContainer = container.getIterateLocked();
        for (auto && element : *lockedContainer)
        {
            volatile int x = 0;
            for (int i=0; i<1000; ++i)
            {
                x += i;
            }
            auto lockedElement = element.getWriteLocked();
            *lockedElement = x;
        }
    }
}


template<class T>
void measure(const char * msg, T test)
{
    auto start = std::chrono::high_resolution_clock::now();
    
    int n = 3000;   
    for (auto i=0; i<n; ++i) 
    {
        test.getWriteLocked()->push_back(1);
    }
    
    std::vector<std::thread> threads;
    int sum = 0;
    
    for (int i=0; i<5; ++i) 
    {
        if (detail::isLockingIterateValue<decltype(test)>) {
            threads.emplace_back(std::thread{[&]() -> void { write(test); }});
        } else {
            write(test);
        }
    }
    
    for (int i=0; i<10; ++i) 
    {
        if (detail::isLockingIterateValue<decltype(test)>)  {
            threads.emplace_back(std::thread{[&]() -> void { update(test); }});
        } else {
            update(test);
        }
    }
    
    for (int i=0; i<20; ++i) 
    {
        if (detail::isLockingIterateValue<decltype(test)>)  {
            threads.emplace_back(std::thread{[&]() -> void { sum += read(test); }});
        } else {
            read(test);
        }
    }
    
    for (auto & thread : threads) { thread.join(); }
    
    auto finish = std::chrono::high_resolution_clock::now();
    std::cout << msg << ": " <<
        std::chrono::duration_cast<std::chrono::microseconds>(finish - start).count() / 10 / 100. 
        << " ms, sum = " << sum << std::endl;
}


int main() 
{    
    Locking<std::map<int, Dummy<int>>, std::mutex> x = {};
    std::cout << typeid(x.getIterateLocked()).name() << std::endl;
         
    measure("single threaded, elements = none,         container = none        ", 
         Dummy<std::vector<Dummy<int>>>{});
    
    measure("multi threaded,  elements = none,         container = mutex       ",
         Locking<std::vector<Dummy<int>>, std::mutex>{});
    
    measure("multi threaded,  elements = none,         container = shared_mutex", 
         Locking<std::vector<Dummy<int>>, std::shared_mutex>{});
    
    measure("multi threaded,  elements = mutex,        container = mutex       ", 
         Locking<std::vector<Locking<int, std::mutex>>, std::mutex>{});
    
    measure("multi threaded,  elements = mutex,        container = shared_mutex", 
         Locking<std::vector<Locking<int, std::mutex>>, std::shared_mutex>{});
    
    measure("multi threaded,  elements = shared_mutex, container = mutex       ", 
         Locking<std::vector<Locking<int, std::shared_mutex>>, std::mutex>{});
    
    measure("multi threaded,  elements = shared_mutex, container = shared_mutex", 
         Locking<std::vector<Locking<int, std::shared_mutex>>, std::shared_mutex>{});
        
    return 0;
}
