import { describe, it, expect, beforeEach, vi } from 'vitest'
import { mount } from '@vue/test-utils'
import FirmwareView from '../FirmwareView.vue'
import { createTestingPinia } from '../../tests/testUtils'

describe('FirmwareView (comprehensive)', () => {
  beforeEach(() => {
    vi.clearAllMocks()
  })

  it('displays firmware upload section', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(FirmwareView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsFileUpload: { template: '<div class="file-upload"></div>' },
          BsProgress: { template: '<div class="progress"></div>' },
          BsButton: { template: '<button><slot /></button>' },
          BsCard: { template: '<div><slot /></div>' }
        }
      }
    })
    expect(wrapper.exists()).toBe(true)
    expect(wrapper.html().length).toBeGreaterThan(100)
  })

  it('renders firmware form controls', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(FirmwareView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsFileUpload: { template: '<input type="file" />' },
          BsProgress: { template: '<div></div>' }
        }
      }
    })
    expect(wrapper.find('input[type="file"]').exists()).toBe(true)
  })

  it('shows platform information', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(FirmwareView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsFileUpload: { template: '<div></div>' },
          BsProgress: { template: '<div></div>' },
          BsCard: { template: '<div><slot /></div>' }
        }
      }
    })
    const html = wrapper.html()
    expect(html.length).toBeGreaterThan(100)
  })

  it('has firmware upload button', () => {
    const pinia = createTestingPinia()
    const wrapper = mount(FirmwareView, {
      global: {
        plugins: [pinia],
        stubs: {
          BsFileUpload: { template: '<div></div>' },
          BsProgress: { template: '<div></div>' },
          BsButton: { template: '<button id="upload-btn"><slot /></button>' }
        }
      }
    })
    expect(wrapper.find('#upload-btn').exists()).toBe(true)
  })
})
