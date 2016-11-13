#include <cstddef>

/**
 * @brief   Define non-existing function to catch the usage of new.
 */
extern void you_tried_to_use_new(void);

/**
 * @brief   Define non-existing function to catch the usage of delete.
 */
extern void you_tried_to_use_delete(void);

/**
 * @brief   Redefinition of new to stop the usage of it.
 */
void* operator new(size_t size) {
  (void)size;
  you_tried_to_use_new();
  return NULL;
}

void* operator new(size_t size, void* ptr) {
  (void)size;
  (void)ptr;
  you_tried_to_use_new();
  return NULL;
}

void* operator new[](size_t size) {
  (void)size;
  you_tried_to_use_new();
  return NULL;
}

void* operator new[](size_t size, void* ptr) {
  (void)size;
  (void)ptr;
  you_tried_to_use_new();
  return NULL;
}

/**
 * @brief   Redefinition of delete to stop the usage of it.
 */
void operator delete(void* ptr) {
  (void)ptr;
  you_tried_to_use_delete();
}

void operator delete(void* ptr, size_t size) {
  (void)size;
  (void)ptr;
  you_tried_to_use_delete();
}

void operator delete[](void* ptr) {
  (void)ptr;
  you_tried_to_use_delete();
}

void operator delete[](void* ptr, size_t size) {
  (void)size;
  (void)ptr;
  you_tried_to_use_delete();
}

