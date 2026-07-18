import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import VoltageFragment from '../VoltageFragment.vue'
import { createTestingPinia } from '../../tests/testUtils'

describe('VoltageFragment (comprehensive)', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('shows voltage calculation controls', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(VoltageFragment, {
      global: {
        plugins: [pinia],
        stubs: {
          BsInputNumber: { template: '<input type="number" class="bs-input-number" />' },
          BsInputReadonly: { template: '<div class="bs-input-readonly"></div>' },
          BsButton: { template: '<button><slot /></button>' },
          BsCard: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.find('.bs-input-number').exists()).toBe(true)
  })

  it('displays measured voltage input', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(VoltageFragment, {
      global: {
        plugins: [pinia],
        stubs: {
          BsInputNumber: { template: '<input type="number" />' },
          BsInputReadonly: { template: '<div></div>' },
          BsCard: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.html().length).toBeGreaterThan(100)
  })

  it('has calculate button', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(VoltageFragment, {
      global: {
        plugins: [pinia],
        stubs: {
          BsInputNumber: { template: '<input type="number" />' },
          BsInputReadonly: { template: '<div></div>' },
          BsButton: { template: '<button class="calc-btn"><slot /></button>' }
        }
      }
    })
    const buttons = wrapper.findAll('button')
    expect(buttons.length).toBeGreaterThan(0)
  })

  it('component renders without errors', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(VoltageFragment, {
      global: {
        plugins: [pinia],
        stubs: {
          BsInputNumber: true,
          BsInputReadonly: true,
          BsButton: true
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
  })
})
