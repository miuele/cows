#ifndef ASSOC_ARENA_HPP_INCLUDED
#define ASSOC_ARENA_HPP_INCLUDED

#include <vector>
#include <unordered_map>
#include <stdexcept>

namespace assoc_arena {

template <class Key, class Value>
class AssociativeArena {
public:
	explicit AssociativeArena(std::size_t size) {
		pool_.reserve(size);
	}

	explicit AssociativeArena(const AssociativeArena &) = default;
	AssociativeArena(AssociativeArena &&) = default;

	std::size_t capacity() const {
		return pool_.capacity();
	}

	std::size_t consumed() const {
		return pool_.size();
	}

	template <class ...Args>
	Value *emplace(const Key &key, Args &&...args) {
		if (pool_.size() >= pool_.capacity()) {
			throw std::out_of_range("no more buffer left");
		}
		pool_.emplace_back(std::forward<Args>(args)...);
		if (!pointers_.try_emplace(key, pool_.size() - 1).second) {
			throw std::out_of_range("associated memory already exists");
		}
		return &pool_.back();
	}

	bool exists(const Key &key) const {
		return pointers_.count(key) == 1;
	}

	Value *get(const Key &key) {
		return &pool_[pointers_.at(key)];
	}

	const Value *get(const Key &key) const {
		return &pool_[pointers_.at(key)];
	}

	std::unordered_map<Key, std::size_t> pointers_;
	std::vector<Value> pool_;
};

}

#endif