#ifndef PARALLEL_Q_H
#define PARALLEL_Q_H

// PARALLEL_Q是一个泛型队列类，支持并行操作。
// 它允许在多线程环境中高效地添加、移除和访问元素，适用于需要并行数据处理的场合。
#include <cassert>

template <typename T>
class PARALLEL_Q
{
private:
    int counter = 0;                    // 记录队列中当前元素的数量
    int Q_LEN;                          // 队列的最大长度
    bool is_empty, initialized = false; // 分别标记队列是否为空和是否已初始化
public:
    T *q;                                   // 动态数组，用于存储队列元素
    int head = 0, tail = 0;                 // 队列头和尾的索引
    PARALLEL_Q();                           // 默认构造函数
    PARALLEL_Q(int len);                    // 参数化构造函数，指定队列长度
    ~PARALLEL_Q();                          // 析构函数，负责资源释放
    void init(int len);                     // 初始化队列，分配资源
    void pop();                             // 从队列头部移除元素
    T front();                              // 返回队列头部元素的引用
    T back();                               // 返回队列尾部元素的引用
    void clear();                           // 清空队列
    void push(T op);                        // 向队列尾部添加元素
    void push_parallel(T &op, int index);   // 并行地向队列的指定位置添加元素
    void push_pos(T &op, int index);        // 向队列的指定位置添加元素，更新计数器和尾索引
    void push_parallel_prepare(int length); // 为并行添加元素做准备，预留空间
    bool empty();                           // 检查队列是否为空
    int size();                             // 返回队列中元素的数量
};
// ------------------------构造函数和析构函数---------------------------------
template <typename T>
PARALLEL_Q<T>::PARALLEL_Q()
{
    initialized = false; // 默认构造函数，初始化时标记队列未初始化
}

template <typename T>
PARALLEL_Q<T>::PARALLEL_Q(int len)
{
    Q_LEN = len;        // 参数化构造函数，根据提供的长度初始化队列
    q = new T[Q_LEN];   // 分配存储空间
    initialized = true; // 标记队列为已初始化
}

template <typename T>
PARALLEL_Q<T>::~PARALLEL_Q()
{
    if (initialized)
        delete[] q; // 析构函数，如果队列已初始化，则释放存储空间
}

// ------------------------队列的基本操作---------------------------------
template <typename T>
void PARALLEL_Q<T>::init(int len)
{
    Q_LEN = len;        // 初始化队列，设置队列长度
    q = new T[Q_LEN];   // 分配存储空间
    initialized = true; // 标记队列为已初始化
}

// 移除队列头
template <typename T>
void PARALLEL_Q<T>::pop()
{
    // 确保队列已初始化
    assert(initialized && "Queue is not initialized!");
    // 如果队列为空，则不执行任何操作
    if (counter == 0)
        return;
    // 移动头指针
    head++;
    // 确保头指针不会超出范围
    head %= Q_LEN;
    // 减少队列中的元素计数
    counter--;
    // 如果移除元素后队列为空，则更新状态
    if (counter == 0)
        is_empty = true;
    return;
}

template <typename T>
T PARALLEL_Q<T>::front()
{
    // 确保队列已初始化
    assert(initialized && "Queue is not initialized!");
    // 返回队列头部的元素
    return q[head];
}

template <typename T>
T PARALLEL_Q<T>::back()
{
    // 确保队列已初始化
    assert(initialized && "Queue is not initialized!");
    // 返回队列尾部的元素
    return q[(tail + Q_LEN - 1) % Q_LEN];
}

template <typename T>
void PARALLEL_Q<T>::clear()
{
    assert(initialized && "Queue is not initialized!");
    head = 0;        // 重置头指针
    tail = 0;        // 重置尾指针
    counter = 0;     // 重置元素计数
    is_empty = true; // 更新队列状态为为空
    return;
}

//
template <typename T>
void PARALLEL_Q<T>::push(T op)
{
    // 确保队列已初始化
    assert(initialized && "Queue is not initialized!");
    if (counter == Q_LEN)
    {
        // 如果队列已满，输出提示并移除头元素
        printf("Queue FULL. Head Element Popped! ");
        pop();
    }
    // 在尾部添加新元素
    q[tail] = op;
    // 增加元素计数
    counter++;
    // 如果队列之前为空，更新状态
    if (is_empty)
        is_empty = false;
    // 移动尾指针
    tail++;
    // 确保尾指针不会超出范围
    tail %= Q_LEN;
}

template <typename T>
void PARALLEL_Q<T>::push_pos(T &op, int index)
{
    // 该函数与push函数类似，但允许直接在指定位置添加元素
    // 适用于知道具体索引位置时的操作，但在实际使用中需注意线程安全和索引合法性
    assert(initialized && "Queue is not initialized!");
    if (counter == Q_LEN)
    {
        // 如果队列已满，则移除头元素
        printf("Queue FULL. Head Element Popped! ");
        pop();
    }
    counter++;
    index %= Q_LEN; // 确保索引不会超出队列长度
    q[index] = op;  // 在指定位置添加元素
    if (is_empty)
        is_empty = false; // 如果队列是空的，更新状态
    tail++;               // 增加尾指针
    tail %= Q_LEN;        // 确保尾指针不会超出队列长度
}

// ------------------------并行操作队列----------------------------
template <typename T>
void PARALLEL_Q<T>::push_parallel(T &op, int index)
{
    // 专为并行操作设计的函数，允许多线程同时在队列的不同位置添加元素
    // 使用时需要确保操作的线程安全，避免索引冲突
    assert(initialized && "Queue is not initialized!");
    if (counter == Q_LEN)
    {
        printf("Queue FULL. Head Element Popped! ");
        pop();
    }
    index %= Q_LEN; // 确保索引不会超出队列长度
    q[index] = op; // 在指定位置添加元素
}

template <typename T>
void PARALLEL_Q<T>::push_parallel_prepare(int length)
{
    // 为并行添加操作准备，预先计算并更新队列的状态。这个方法用于在实际添加元素之前，
    // 预留足够的空间以避免在并行添加时出现队列已满的情况。
    assert(initialized && "Queue is not initialized!");
    assert(counter + length < Q_LEN && "Push Length is out of Queue Capacity!");
    if (counter == Q_LEN)
    {
        printf("Queue FULL. Head Element Popped! ");
        pop();
    }
    counter += length; // 更新元素计数
    if (is_empty)
        is_empty = false; // 如果队列是空的，更新状态
    tail += length;       // 更新尾指针
    tail %= Q_LEN;        // 确保尾指针不会超出队列长度
}

// ---------------------------状态查询----------------------------
// 检查队列是否为空。
template <typename T>
bool PARALLEL_Q<T>::empty()
{
    // 确保队列已初始化
    assert(initialized && "Queue is not initialized!");
    return is_empty; // 返回队列是否为空的状态
}

// 返回队列中的元素数量。
template <typename T>
int PARALLEL_Q<T>::size()
{
    // 确保队列已初始化
    assert(initialized && "Queue is not initialized!");
    return counter; // 返回元素计数
}

#endif