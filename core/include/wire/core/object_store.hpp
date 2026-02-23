#pragma once

#include <concepts>
#include <cstddef>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include "wire/core/id.hpp"

namespace wire::core {

template <typename T>
concept HasObjectId = requires(T value) {
  { value.id } -> std::convertible_to<ObjectId>;
};

template <HasObjectId T>
class ObjectStore {
 public:
  ObjectStore() = default;

  [[nodiscard]] std::size_t size() const { return items_.size(); }

  [[nodiscard]] bool empty() const { return items_.empty(); }

  [[nodiscard]] bool contains(ObjectId id) const { return index_by_id_.contains(id); }

  [[nodiscard]] std::optional<std::size_t> index_of(ObjectId id) const {
    auto it = index_by_id_.find(id);
    if (it == index_by_id_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  [[nodiscard]] T* at_index(std::size_t index) {
    if (index >= items_.size()) {
      return nullptr;
    }
    return &items_[index];
  }

  [[nodiscard]] const T* at_index(std::size_t index) const {
    if (index >= items_.size()) {
      return nullptr;
    }
    return &items_[index];
  }

  [[nodiscard]] T* find(ObjectId id) {
    auto it = index_by_id_.find(id);
    if (it == index_by_id_.end()) {
      return nullptr;
    }
    return &items_[it->second];
  }

  [[nodiscard]] const T* find(ObjectId id) const {
    auto it = index_by_id_.find(id);
    if (it == index_by_id_.end()) {
      return nullptr;
    }
    return &items_[it->second];
  }

  T& insert(const T& value) {
    auto it = index_by_id_.find(value.id);
    if (it != index_by_id_.end()) {
      items_[it->second] = value;
      return items_[it->second];
    }
    items_.push_back(value);
    index_by_id_[value.id] = items_.size() - 1;
    return items_.back();
  }

  T& insert(T&& value) {
    const ObjectId id = value.id;
    auto it = index_by_id_.find(id);
    if (it != index_by_id_.end()) {
      items_[it->second] = std::move(value);
      return items_[it->second];
    }
    items_.push_back(std::move(value));
    index_by_id_[id] = items_.size() - 1;
    return items_.back();
  }

  bool remove(ObjectId id) {
    auto it = index_by_id_.find(id);
    if (it == index_by_id_.end()) {
      return false;
    }

    const std::size_t remove_index = it->second;
    const std::size_t last_index = items_.size() - 1;

    if (remove_index != last_index) {
      items_[remove_index] = std::move(items_[last_index]);
      index_by_id_[items_[remove_index].id] = remove_index;
    }

    items_.pop_back();
    index_by_id_.erase(it);
    return true;
  }

  void clear() {
    items_.clear();
    index_by_id_.clear();
  }

  [[nodiscard]] const std::vector<T>& items() const { return items_; }
  [[nodiscard]] std::vector<T>& items() { return items_; }

 private:
  std::vector<T> items_;
  std::unordered_map<ObjectId, std::size_t> index_by_id_;
};

}  // namespace wire::core
