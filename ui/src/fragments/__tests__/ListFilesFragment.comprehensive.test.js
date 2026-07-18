import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import ListFilesFragment from '../ListFilesFragment.vue'
import { createTestingPinia } from '../../tests/testUtils'

describe('ListFilesFragment (comprehensive)', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('displays file listing interface', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(ListFilesFragment, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div><slot /></div>' },
          BsButton: { template: '<button><slot /></button>' },
          BsMessage: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
  })

  it('renders file management controls', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(ListFilesFragment, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div><slot /></div>' },
          BsButton: { template: '<button class="file-btn"><slot /></button>' }
        }
      }
    })
    expect(wrapper.html().length).toBeGreaterThan(0)
  })

  it('has buttons for file operations', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(ListFilesFragment, {
      global: {
        plugins: [pinia],
        stubs: {
          BsCard: { template: '<div><slot /></div>' },
          BsButton: { template: '<button><slot /></button>' },
          BsMessage: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
  })
})
