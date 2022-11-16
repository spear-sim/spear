// #ifdef _MSC_VER
    #if PLATFORM_64BITS
        #define INT                               ::INT
        #define UINT                              ::UINT
        #define DWORD                             ::DWORD
        #define FLOAT                             ::FLOAT
        #define TRUE                              1
        #define FALSE                             0
        #define InterlockedIncrement              _InterlockedIncrement
        #define InterlockedDecrement              _InterlockedDecrement
        #define InterlockedAdd                    _InterlockedAdd
        #define InterlockedExchange               _InterlockedExchange
        #define InterlockedExchangeAdd            _InterlockedExchangeAdd
        #define InterlockedCompareExchange        _InterlockedCompareExchange
        #define InterlockedAnd                    _InterlockedAnd
        #define InterlockedOr                     _InterlockedOr
        #define InterlockedXor                    _InterlockedXor
        #define InterlockedCompareExchangePointer _InterlockedCompareExchangePointer
        #define InterlockedExchange64             _InterlockedExchange64
        #define InterlockedExchangeAdd64          _InterlockedExchangeAdd64
        #define InterlockedCompareExchange64      _InterlockedCompareExchange64
        #define InterlockedIncrement64            _InterlockedIncrement64
        #define InterlockedDecrement64            _InterlockedDecrement64
    #else
        #define INT                               ::INT
        #define UINT                              ::UINT
        #define DWORD                             ::DWORD
        #define FLOAT                             ::FLOAT
        #define TRUE                              1
        #define FALSE                             0
        #define InterlockedIncrement              _InterlockedIncrement
        #define InterlockedDecrement              _InterlockedDecrement
        #define InterlockedAdd                    _InterlockedAdd
        #define InterlockedExchange               _InterlockedExchange
        #define InterlockedExchangeAdd            _InterlockedExchangeAdd
        #define InterlockedCompareExchange        _InterlockedCompareExchange
        #define InterlockedAnd                    _InterlockedAnd
        #define InterlockedOr                     _InterlockedOr
        #define InterlockedXor                    _InterlockedXor
        #define InterlockedCompareExchangePointer __InlineInterlockedCompareExchangePointer
    #endif
// #endif
