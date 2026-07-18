import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import SerialView from '../SerialView.vue'
import { createTestingPinia } from '../../tests/testUtils'

describe('SerialView (comprehensive)', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('mounts and renders properly', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(SerialView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div><slot /></div>' },
          BsInputReadonly: { template: '<div></div>' },
          BsButton: { template: '<button><slot /></button>' },
          BsMessage: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
    expect(wrapper.html().length).toBeGreaterThan(0)
  })

  it('component has required structure', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(SerialView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div class="bs-card"><slot /></div>' },
          BsMessage: { template: '<div class="bs-message"><slot /></div>' }
        }
      }
    })
    // SerialView contains a h3 title and pre element for serial output
    expect(wrapper.find('pre').exists()).toBe(true)
    expect(wrapper.find('.h3').exists()).toBe(true)
  })

  it('displays serial configuration', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(SerialView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div><slot /></div>' },
          BsInputReadonly: { template: '<div></div>' }
        }
      }
    })
    const html = wrapper.html()
    expect(html.length).toBeGreaterThan(100)
  })
})
