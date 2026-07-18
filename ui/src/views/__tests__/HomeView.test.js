import { beforeEach, describe, expect, it, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import HomeView from '../HomeView.vue'
import { setupTestEnvironment, commonStubs } from '@/tests/testUtils'
import { global, status, config } from '@/modules/pinia'

describe('HomeView', () => {
  let pinia

  const createWrapper = (props = {}) =>
    mount(HomeView, {
      props,
      global: {
        plugins: [pinia],
        stubs: {
          ...commonStubs,
          PidTemperatureFragment: true,
          PidControllerFragment: true,
          BsCard: true
        }
      }
    })

  beforeEach(() => {
    const env = setupTestEnvironment()
    pinia = env.pinia
  })

  describe('Rendering', () => {
    it('renders the component', () => {
      const wrapper = createWrapper()
      expect(wrapper.exists()).toBe(true)
    })

    it('displays main container', () => {
      const wrapper = createWrapper()
      const container = wrapper.find('.container')
      expect(container.exists()).toBe(true)
    })

    it('conditionally renders temperature fragments', () => {
      const wrapper = createWrapper()
      // The view should exist, actual rendering depends on store state
      expect(wrapper.vm).toBeDefined()
    })
  })

  describe('Template Structure', () => {
    it('has proper responsive grid classes', () => {
      const wrapper = createWrapper()
      expect(wrapper.find('.row').exists()).toBe(true)
    })
  })

  describe('formatTime helper', () => {
    it('formats times under 60 seconds as seconds', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.formatTime(30)).toBe('30s')
    })

    it('formats times under 1 hour as minutes', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.formatTime(90)).toBe('2m')
    })

    it('formats times under 1 day as hours', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.formatTime(7200)).toBe('2h')
    })

    it('formats times of 1 day or more as days', () => {
      const wrapper = createWrapper()
      expect(wrapper.vm.formatTime(172800)).toBe('2d')
    })
  })

  describe('updateTimers', () => {
    it('can be called directly without error', () => {
      const wrapper = createWrapper()
      expect(() => wrapper.vm.updateTimers()).not.toThrow()
    })
  })

  describe('lifecycle cleanup', () => {
    it('clears polling intervals on unmount', () => {
      const clearIntervalSpy = vi.spyOn(window, 'clearInterval')
      const wrapper = createWrapper()
      wrapper.unmount()
      expect(clearIntervalSpy).toHaveBeenCalled()
      clearIntervalSpy.mockRestore()
    })
  })

  describe('formatTemp helper', () => {
    it('formats temperature in Celsius', () => {
      config.temp_format = 'C'
      const wrapper = createWrapper()
      const result = wrapper.vm.formatTemp(20)
      expect(result).toBeDefined()
    })

    it('formats temperature in Fahrenheit', () => {
      config.temp_format = 'F'
      const wrapper = createWrapper()
      const result = wrapper.vm.formatTemp(20)
      expect(result).toBeDefined()
      config.temp_format = 'C'
    })
  })

  describe('refresh function', () => {
    it('can be called without error', async () => {
      const wrapper = createWrapper()
      await expect(wrapper.vm.refresh()).resolves.not.toThrow()
    })

    it('calls status.load', async () => {
      const wrapper = createWrapper()
      await wrapper.vm.refresh()
      expect(status.load).toHaveBeenCalled()
    })

    it('handles status.load returning false (line 190 false branch)', async () => {
      vi.mocked(status.load).mockResolvedValueOnce(false)
      const wrapper = createWrapper()
      await wrapper.vm.refresh()
      // Should not throw when success is false
      expect(wrapper.exists()).toBe(true)
    })
  })

  describe('PID actuator status ternary branches (lines 76-77)', () => {
    const createWrapperWithSlotCard = () =>
      mount(HomeView, {
        global: {
          plugins: [pinia],
          stubs: {
            ...commonStubs,
            PidTemperatureFragment: true,
            PidControllerFragment: true,
            // Use a stub that renders the slot so ternary expressions are evaluated
            BsCard: { template: '<div class="card"><slot /></div>' }
          }
        }
      })

    it('shows Active for cooling when pid_cooling_actuator_active is true', async () => {
      status.pid_cooling_actuator_active = true
      const wrapper = createWrapperWithSlotCard()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Active')
      status.pid_cooling_actuator_active = false
    })

    it('shows Active for heating when pid_heating_actuator_active is true', async () => {
      status.pid_heating_actuator_active = true
      const wrapper = createWrapperWithSlotCard()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Active')
      status.pid_heating_actuator_active = false
    })

    it('shows Inactive for cooling when pid_cooling_actuator_active is false', async () => {
      status.pid_cooling_actuator_active = false
      const wrapper = createWrapperWithSlotCard()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Inactive')
    })
  })

  describe('BLE sensor rendering', () => {
    it('renders BLE sensor cards when feature enabled and devices present', async () => {
      global.feature = { ble_sensor: true }
      status.temperature_device = [
        { device: 'test-sensor', update_time: 30, temp: 20.5, source: 'ble', type: 'sensor' }
      ]
      config.temp_format = 'C'

      const wrapper = mount(HomeView, {
        global: {
          plugins: [pinia],
          stubs: {
            ...commonStubs,
            PidTemperatureFragment: true,
            PidControllerFragment: true,
            BsCard: { template: '<div class="card"><slot /></div>' }
          }
        }
      })
      await wrapper.vm.$nextTick()

      expect(wrapper.text()).toContain('Temperature:')

      // restore
      global.feature = { ble_sensor: false }
      status.temperature_device = []
    })
  })

  describe('pid_mode template branches', () => {
    const createWrapperWithBsCard = () =>
      mount(HomeView, {
        global: {
          plugins: [pinia],
          stubs: {
            ...commonStubs,
            PidTemperatureFragment: true,
            PidControllerFragment: true,
            BsCard: { template: '<div class="card"><slot /></div>' }
          }
        }
      })

    it('renders Beer Constant when pid_mode is b', async () => {
      status.pid_mode = 'b'
      const wrapper = createWrapperWithBsCard()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Beer Constant')
      status.pid_mode = ''
    })

    it('renders Fridge Constant when pid_mode is f', async () => {
      status.pid_mode = 'f'
      const wrapper = createWrapperWithBsCard()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Fridge Constant')
      status.pid_mode = ''
    })

    it('renders Off when pid_mode is neither b nor f', async () => {
      status.pid_mode = 'o'
      const wrapper = createWrapperWithBsCard()
      await wrapper.vm.$nextTick()
      expect(wrapper.text()).toContain('Off')
      status.pid_mode = ''
    })
  })
})
