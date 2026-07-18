import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import DevicePidView from '../DevicePidView.vue'
import { createTestingPinia } from '../../tests/testUtils'

describe('DevicePidView', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('mounts without error', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(DevicePidView, {
      global: {
        plugins: [pinia],
        stubs: {
          PidControllerFragment: true,
          PidDataFragment: true,
          PidTemperatureFragment: true,
          BsCard: true
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
  })
})
