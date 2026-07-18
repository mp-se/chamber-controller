import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import PidTemperatureFragment from '../PidTemperatureFragment.vue'
import { status, config } from '@/modules/pinia'

describe('PidTemperatureFragment', () => {
  beforeEach(() => {
    vi.clearAllMocks()
    // Reset singleton store state for isolation
    status.pid_state = 0
    status.pid_mode = ''
    status.pid_beer_temp_connected = false
    status.pid_fridge_temp_connected = false
    status.pid_beer_temp = 0
    status.pid_fridge_temp = 0
    status.pid_beer_target_temp = 0
    status.pid_fridge_target_temp = 0
    status.pid_time_since_cooling = 0
    status.pid_time_since_heating = 0
    status.pid_time_since_idle = 0
    status.pid_wait_time = 0
    config.temp_format = 'C'
  })

  const createWrapper = () =>
    mount(PidTemperatureFragment, {
      global: { stubs: {} }
    })

  describe('component mounting', () => {
    it('mounts without error', () => {
      const wrapper = createWrapper()
      expect(wrapper.exists()).toBe(true)
    })

    it('displays Controller header', () => {
      const wrapper = createWrapper()
      expect(wrapper.text()).toContain('Controller')
    })

    it('displays Beer and Chamber labels', () => {
      const wrapper = createWrapper()
      expect(wrapper.text()).toContain('Beer:')
      expect(wrapper.text()).toContain('Chamber:')
    })
  })

  describe('beerTemp computed', () => {
    it('returns dashes when beer temp sensor not connected', async () => {
      status.pid_beer_temp_connected = false
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('--')
    })

    it('returns temperature value when beer temp sensor connected', async () => {
      status.pid_beer_temp_connected = true
      status.pid_beer_temp = 20
      config.temp_format = 'C'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('°C')
    })

    it('formats beer temp with Fahrenheit format', async () => {
      status.pid_beer_temp_connected = true
      status.pid_beer_temp = 18
      config.temp_format = 'F'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('°F')
    })
  })

  describe('fridgeTemp computed', () => {
    it('returns dashes when fridge temp sensor not connected', async () => {
      status.pid_fridge_temp_connected = false
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      // Should contain -- somewhere for the unconnected sensor
      const text = wrapper.text()
      expect(text).toContain('Chamber:')
    })

    it('returns temperature value when fridge temp sensor connected', async () => {
      status.pid_fridge_temp_connected = true
      status.pid_fridge_temp = 5
      config.temp_format = 'C'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('°C')
    })
  })

  describe('mode computed', () => {
    it('returns Off when pid_mode is empty', async () => {
      status.pid_mode = ''
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Off')
    })

    it('returns Chamber constant when pid_mode is f', async () => {
      status.pid_mode = 'f'
      status.pid_fridge_target_temp = 5
      config.temp_format = 'C'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Chamber constant')
    })

    it('returns Beer constant when pid_mode is b', async () => {
      status.pid_mode = 'b'
      status.pid_beer_target_temp = 18
      config.temp_format = 'C'
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Beer constant')
    })
  })

  describe('state computed - all pid_state values', () => {
    it('returns Idle state (case 0) when cooling time is less than heating time', async () => {
      status.pid_state = 0
      status.pid_time_since_cooling = 100
      status.pid_time_since_heating = 200
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Idle for')
    })

    it('returns Idle state (case 0) when heating time is less than cooling time', async () => {
      status.pid_state = 0
      status.pid_time_since_cooling = 200
      status.pid_time_since_heating = 100
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Idle for')
    })

    it('returns Off for state case 1', async () => {
      status.pid_state = 1
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Off')
    })

    it('returns Heating for state case 2', async () => {
      status.pid_state = 2
      status.pid_time_since_idle = 600
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Heating for')
    })

    it('returns Cooling for state case 3', async () => {
      status.pid_state = 3
      status.pid_time_since_idle = 600
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Cooling for')
    })

    it('returns Waiting to cool for state case 4', async () => {
      status.pid_state = 4
      status.pid_wait_time = 300
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Waiting to cool')
    })

    it('returns Waiting to heat for state case 5', async () => {
      status.pid_state = 5
      status.pid_wait_time = 300
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Waiting to heat')
    })

    it('returns Waiting for peek detect for state case 6', async () => {
      status.pid_state = 6
      status.pid_wait_time = 300
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Waiting for peek detect')
    })

    it('returns Cooling min time for state case 7', async () => {
      status.pid_state = 7
      status.pid_time_since_idle = 600
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Cooling min time')
    })

    it('returns Heating min time for state case 8', async () => {
      status.pid_state = 8
      status.pid_time_since_idle = 600
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Heating min time')
    })

    it('returns Unknown status for unrecognized state', async () => {
      status.pid_state = 99
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Unknown status')
    })
  })

  describe('reactivity', () => {
    it('updates rendered text when pid_state changes', async () => {
      status.pid_state = 1
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Off')

      status.pid_state = 2
      status.pid_time_since_idle = 60
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Heating for')
    })

    it('updates rendered text when pid_mode changes', async () => {
      status.pid_mode = ''
      const wrapper = createWrapper()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Off')

      status.pid_mode = 'f'
      status.pid_fridge_target_temp = 4
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Chamber constant')
    })
  })
})
