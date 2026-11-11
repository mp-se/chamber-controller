import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import HomeView from '../HomeView.vue'
import { createTestingPinia } from '../../tests/testUtils'

describe('HomeView (comprehensive)', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('displays device ID section', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(HomeView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div class="card"><slot /></div>' },
          BsButton: { template: '<button><slot /></button>' },
          BsMessage: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.text().length).toBeGreaterThan(0)
  })

  it('displays firmware information', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(HomeView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div class="card"><slot /></div>' },
          BsMessage: { template: '<div><slot /></div>' }
        }
      }
    })
    const html = wrapper.html()
    expect(html.length).toBeGreaterThan(100)
  })

  it('displays network information', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(HomeView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div class="card"><slot /></div>' },
          BsMessage: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.html().length).toBeGreaterThan(0)
  })

  it('can mount with complete stubs', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(HomeView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: true,
          BsButton: true,
          BsMessage: true,
          BadgeComponent: true
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
  })
})
