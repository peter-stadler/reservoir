#include <shared_mutex>

namespace {

// traits:

template<class Mutex>
struct WriteLock
{
    using Type = std::unique_lock<Mutex>;
};


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


template<typename Type, class Mutex, class ReadLock, class WriteLock, class>
class LockingHelper;


template<typename T>
struct isLockingHelper : std::false_type {};

template <typename... Args>
struct isLockingHelper<LockingHelper<Args...>> : std::true_type{};

template <typename... Args>
struct isLockingHelper<LockingHelper<Args...> &> : std::true_type{};

template <typename... Args>
struct isLockingHelper<const LockingHelper<Args...>> : std::true_type{};

template <typename... Args>
struct isLockingHelper<const LockingHelper<Args...> &> : std::true_type{};

template<typename T>
constexpr bool isLocking = isLockingHelper<T>::value;

// access locked element:

template<typename Type, class Lock>
class Locked;

template<typename Type, class Lock>
Type & operator*(Locked<Type, Lock> & lvalue) 
{
    return lvalue.element;
}


template<typename Type, class Lock>
class DereferencedForAssignment;

template<typename Type, class Lock>
DereferencedForAssignment<Type, Lock> operator*(Locked<Type, Lock> && rvalue) 
{
    return {std::move(rvalue)};
}


template<typename Type, class Lock>
class DereferencedForAssignment
{
public:
    
    Type operator=(Type element)
    {
        lvalue.element = std::move(element);
        return lvalue.element;
    }
    
private:
    
    Locked<Type, Lock> & lvalue;
    
    DereferencedForAssignment(Locked<Type, Lock> && rvalue) : lvalue(rvalue)
    {}   
    
    friend DereferencedForAssignment<Type, Lock> operator*<Type, Lock>(Locked<Type, Lock> && rvalue);
};


// passthrough to element holding locked mutex:

template<typename Type, class Lock>
class Locked
{
public:
    
    Type * operator->()
    {
        return &element;
    }

//     would allow rvalue this! -> use friend accepting only lvalue
//     Type & operator*()
//     {
//         return element;
//     }
    
    Locked() = delete;
    
    Locked(const Locked & other) = delete;
    Locked(Locked & other) = delete;
    Locked(volatile Locked & other) = delete;    
    Locked(const volatile Locked & other) = delete;
    
    Locked(Locked && other) = default;
    
    ~Locked()
    {
//         if (std::is_same<Lock, std::shared_lock<std::shared_timed_mutex>>::value) { std::cout << "read "; }
//         if (std::is_same<Lock, std::unique_lock<std::shared_timed_mutex>>::value) { std::cout << "write "; }
//         if (std::is_const<Type>::value) { std::cout << "const "; }
//         std::cout << "unlocking " << std::endl;
    }
    
private:
    
    Type & element;
    
    Lock lock;
    
    template<typename Caller>
    Locked(Caller & caller) : 
        element{caller.element},
        lock{*caller.mutex}
    {
//         if (std::is_same<Lock, std::shared_lock<std::shared_timed_mutex>>::value) { std::cout << "read "; }
//         if (std::is_same<Lock, std::unique_lock<std::shared_timed_mutex>>::value) { std::cout << "write "; }
//         if (std::is_const<Type>::value) { std::cout << "const "; }
//         std::cout << "locked " <<std::endl;
    }
    
    template<typename OtherType, class Mutex, class ReadLock, class WriteLock, class>
    friend class LockingHelper;
    
    // Using the member operator* would allow e.g.: auto elem = *x.getReadLocked(); 
    // As the temporary is destroyed immediately after this line, elem would have unlocked access!
    // To prevent this we accept only lvalues for access by the following non-member operator*
    friend Type & operator*<Type, Lock>(Locked<Type, Lock> & lvalue);
    
    // instead of: friend Type & operator*(Locked<Type, Lock> && rvalue);
    // use: DereferencedForAssignment<Type, Lock> operator*(Locked<Type, Lock> && rvalue) 
    friend DereferencedForAssignment<Type, Lock>;
};



// container for element together with its mutex:

template<typename Type, class Mutex, class ReadLock, class WriteLock, typename = void>
class LockingHelper
{
    using ReadLocked = Locked<const Type, ReadLock>;
    
    using WriteLocked = Locked<Type, WriteLock>;
    
public:
    using type = Type;
    
    template<typename...Args>
    LockingHelper(Args...args) : 
        element{args...},
        mutex{new Mutex}
    {}
        
    ReadLocked getReadLocked() const
    {
        return {*this};
    }
    
    WriteLocked getWriteLocked()
    {
        return {*this};
    }
    
private:
    
    Type element;
    
    mutable std::unique_ptr<Mutex> mutex;
    
    template<typename OtherType, class Lock>
    friend class Locked;
};


template<typename Type, class Mutex, class ReadLock, class WriteLock>
class LockingHelper<Type, Mutex, ReadLock, WriteLock, std::void_t<decltype(*std::cbegin(std::declval<Type>()))>> :
    public LockingHelper<Type, Mutex, ReadLock, WriteLock, int>
{
    using UpdateLock = std::conditional_t<isLocking<typename Type::value_type>, ReadLock, WriteLock>;
    
    using UpdateLocked = Locked<Type, UpdateLock>;
    
public:
    
    template<typename...Args>
    LockingHelper(Args...args) 
    {}
    
    UpdateLocked getUpdateLocked()
    {
        return {*this};
    }
    
};

} // namespace


template<typename Type, 
    class Mutex = std::shared_timed_mutex, 
    class ReadLock = typename ReadLock<Mutex>::Type, 
    class WriteLock = typename WriteLock<Mutex>::Type>
using Locking = LockingHelper<Type, Mutex, ReadLock, WriteLock>;



#include <iostream> 
#include <thread>
#include <vector>
#include <chrono>


template<typename Type>
class Dummy
{
private:
    Type element;
    
public:
    using type = Type;
    
    template<typename...Args>
    Dummy(Args...args) : 
        element{args...}
    {}
        
    const Type * getReadLocked() const
    {
        return &element;
    }
    
    Type * getUpdateLocked()
    {
        return &element;
    }
    
    Type * getWriteLocked()
    {
        return &element;
    }
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
            volatile int x;
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
    using ElementLockType = typename T::type::value_type;
    for (int unused=0; unused < 4; ++unused)
    {
        auto lockedContainer = container.getUpdateLocked();
        for (auto && element : *lockedContainer)
        {
            volatile int x;
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
        if (isLocking<decltype(test)>) {
            threads.emplace_back(std::thread{[&]() -> void { write(test); }});
        } else {
            write(test);
        }
    }
    
    for (int i=0; i<10; ++i) 
    {
        if (isLocking<decltype(test)>)  {
            threads.emplace_back(std::thread{[&]() -> void { update(test); }});
        } else {
            update(test);
        }
    }
    
    for (int i=0; i<20; ++i) 
    {
        if (isLocking<decltype(test)>)  {
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
