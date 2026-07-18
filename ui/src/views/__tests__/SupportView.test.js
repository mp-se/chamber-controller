import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import SupportView from '../SupportView.vue'
import { createTestingPinia } from '../../tests/testUtils'

describe('SupportView', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('mounts without error', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(SupportView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsMessage: true
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
  })

  it('displays support information', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(SupportView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsMessage: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.text().length).toBeGreaterThan(0)
  })
})
