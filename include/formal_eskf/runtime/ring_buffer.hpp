// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace formal_eskf::runtime
{

/**
 * Single-owner history with PX4-style access and fixed backing storage.
 * Sample is a bounded, allocation-free value with uint64_t time_us. The caller
 * admits strictly increasing, nonzero timestamps and handles pending-history
 * loss before pushing into a full buffer. Payload copies must not throw or block.
 * See PX4-TIME in docs/spec/requirements/integration/px4/timestamps.md for the IMU,
 * observation and output-history caller contracts. No clock or scheduler is owned.
 */
template <typename Sample, std::size_t Capacity> class RingBuffer
{
    static_assert(Capacity > 0);

public:
    static constexpr std::uint64_t default_max_age_us = 100000;

    /** Invalid lengths leave valid() false. Storage is never allocated on the heap. */
    explicit RingBuffer(std::size_t length) { (void)allocate(length); }
    RingBuffer() = delete;
    RingBuffer(RingBuffer const &) = delete;
    RingBuffer & operator=(RingBuffer const &) = delete;
    RingBuffer(RingBuffer &&) = delete;
    RingBuffer & operator=(RingBuffer &&) = delete;

    /** Startup/restart only: choose 1..Capacity slots. A changed length clears history. */
    [[nodiscard]] bool allocate(std::size_t length);
    bool valid() const { return m_length != 0; }

    /** Requires valid(). Full histories overwrite the oldest slot, as in PX4. */
    void push(Sample const & sample);

    /**
     * Newest sample at/before timestamp, with age strictly below max_age_us.
     * Retire it and all older entries; keep future entries. Also retire expired
     * entries on a miss. False preserves output. Null output preserves history.
     * Output must not alias buffer storage. The default window matches PX4.
     */
    [[nodiscard]] bool pop_first_older_than(std::uint64_t const & timestamp, Sample * output,
                                            std::uint64_t max_age_us = default_max_age_us);

    /** Non-consuming views; require entries() > 0. Mutation invalidates saved views. */
    Sample const & get_oldest() const { return m_samples[m_oldest]; }
    Sample const & get_newest() const { return m_samples[previous(m_write)]; }

    /** Physical slot, index < get_length(). Modify payloads only, never timestamps. */
    Sample & operator[](std::size_t index) { return m_samples[index]; }
    Sample const & operator[](std::size_t index) const { return m_samples[index]; }

    std::size_t get_length() const { return m_length; }
    std::size_t get_oldest_index() const { return m_oldest; }
    std::size_t entries() const { return m_count; }
    /** Logical occupied bytes; unused backing storage remains reserved. */
    std::size_t get_used_size() const { return sizeof(*this) - sizeof(m_samples) + m_count * sizeof(Sample); }
    std::size_t get_total_size() const { return sizeof(*this); }

    /** Clear every slot and index, retaining the configured length. */
    void reset();

private:
    std::size_t next(std::size_t index) const { return index + 1 == m_length ? 0 : index + 1; }
    std::size_t previous(std::size_t index) const { return index == 0 ? m_length - 1 : index - 1; }

    std::array<Sample, Capacity> m_samples{};
    std::size_t m_length = 0;
    std::size_t m_oldest = 0;
    std::size_t m_write = 0;
    std::size_t m_count = 0;
}; /* end class RingBuffer */

template <typename Sample, std::size_t Capacity> bool RingBuffer<Sample, Capacity>::allocate(std::size_t length)
{
    if (length == 0 || length > Capacity)
    {
        return false;
    }
    if (length != m_length)
    {
        m_length = length;
        reset();
    }
    return true;
}

template <typename Sample, std::size_t Capacity> void RingBuffer<Sample, Capacity>::push(Sample const & sample)
{
    m_samples[m_write] = sample;
    m_write = next(m_write);
    if (m_count == m_length)
    {
        m_oldest = next(m_oldest);
    }
    else
    {
        ++m_count;
    }
}

template <typename Sample, std::size_t Capacity>
bool RingBuffer<Sample, Capacity>::pop_first_older_than(std::uint64_t const & timestamp, Sample * output,
                                                        std::uint64_t max_age_us)
{
    if (output == nullptr)
    {
        return false;
    }
    bool selected = false;
    std::size_t removed = 0;
    std::size_t index = m_write;
    for (std::size_t inspected = 0; inspected < m_count; ++inspected)
    {
        index = previous(index);
        auto const time_us = m_samples[index].time_us;
        if (time_us <= timestamp)
        {
            // Ordered subtraction avoids timestamp + age overflow near UINT64_MAX.
            if (timestamp - time_us < max_age_us)
            {
                *output = m_samples[index];
                selected = true;
            }
            removed = m_count - inspected;
            break;
        }
    }
    for (std::size_t retired = 0; retired < removed; ++retired)
    {
        m_samples[m_oldest].time_us = 0;
        m_oldest = next(m_oldest);
    }
    m_count -= removed;
    return selected;
}

template <typename Sample, std::size_t Capacity> void RingBuffer<Sample, Capacity>::reset()
{
    m_samples.fill(Sample{});
    m_oldest = 0;
    m_write = 0;
    m_count = 0;
}

} /* end namespace formal_eskf::runtime */
