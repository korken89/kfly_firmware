#include <cstddef>
#include <new>

/**
 * @brief   Define non-existing function to catch the usage of new.
 */
void you_tried_to_use_new(void);

/**
 * @brief   Define non-existing function to catch the usage of delete.
 */
void you_tried_to_use_delete(void);

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void *operator new(std::size_t size)
{
    (void) size;
    you_tried_to_use_new();
    return NULL;
}

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void *operator new(std::size_t size, std::nothrow_t const &) noexcept
{
    (void) size;
    you_tried_to_use_new();
    return NULL;
}

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void *operator new [](std::size_t size)
{
    (void) size;
    you_tried_to_use_new();
    return NULL;
}

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void *operator new [] (std::size_t size, std::nothrow_t const &) noexcept
{
    (void) size;
    you_tried_to_use_new();
    return NULL;
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void *p)
{
    (void)p;
    you_tried_to_use_delete();
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void *p, std::nothrow_t const &) noexcept
{
    (void)p;
    you_tried_to_use_delete();
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void *p, std::size_t size)
{
    (void)p;
    (void)size;
    you_tried_to_use_delete();
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void *p, std::size_t size, std::nothrow_t const &) noexcept
{
    (void)p;
    (void)size;
    you_tried_to_use_delete();
}
